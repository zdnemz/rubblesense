// fw_replay - a virtual RubbleSense device on the host: the real firmware sources (src/*.cpp) under the virtual
// clock of the test stubs, fed with an I/Q byte stream.
//
// Offline (deterministic, as fast as the CPU allows; this is what tools/test_tools.py uses):
//
//   fw_replay --stream scene.bin --cmd-at 500 "src serial" --cmd-at 3500 scan --until-results 1
//
//   The stream is what `tools/send_iq_replay.py --out scene.bin` writes: rs_proto packets, optionally with console
//   lines in between. Packets reach the device's serial port at the times their own t_ms stamps give (frames
//   without stamps: seq / frame rate) and at the UART's byte rate. Everything the device prints goes to stdout,
//   like on the USB serial port; progress and summary lines go to stderr. Console commands come from --cmd /
//   --cmd-at (times are device time in ms; --cmd alone = 2 s after the previous command, the first at 500 ms).
//   Buttons are pressed with --press-at MS scan|mode|menu (90 ms) or --long-press-at (1.1 s): the result and
//   message screens (e.g. after `calib`) are closed with the menu button, exactly as on the device.
//
// Online (a virtual device on a loopback TCP port, for `send_iq_replay.py --tcp`; UART0 build only):
//
//   fw_replay --listen 7000 --speed 8
//   python3 tools/send_iq_replay.py --file scene.csv --tcp 127.0.0.1:7000 --speed 8
//           --cmd "src serial" --cmd scan --capture scans.jsonl --until-result 1
//
//   The device clock runs at --speed x real time. The server only listens on 127.0.0.1.
//
// Exit status: 0 = ran to the end (and produced --until-results scan records, if asked), 3 = no / too few scan
// records, 2 = usage, 1 = other error.
//
// A MODEL, NOT HARDWARE: the stubs in test/stubs stand in for the Arduino-ESP32 core, the display, the IMU and
// flash. A replay that ends in the right verdict says the firmware logic handles that stream. It says nothing
// about an ESP32's timing, stack or drivers, and nothing about a real radar.
#include <stdint.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include "fw_rig.h"
#include "rs_build.h"
#include "rs_proto.h"

#if defined(__unix__) || defined(__APPLE__)
#define RS_REPLAY_SOCKETS 1
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#else
#define RS_REPLAY_SOCKETS 0
#endif

int g_pass = 0;           // required by rs_test.h; not used here
int g_fail = 0;

namespace {

#if RS_IQ_ON_UART0
HardwareSerial& iqPort() { return Serial; }
constexpr uint32_t kIqBaud = RS_SERIAL_BAUD;
constexpr bool kIqOnConsole = true;
#else
HardwareSerial& iqPort() { return Serial2; }
constexpr uint32_t kIqBaud = RS_IQ_BAUD;
constexpr bool kIqOnConsole = false;
#endif

// ---------------------------------------------------------------------------------------------- stream
enum class Kind : uint8_t { kFrame, kInfo, kJunk };

struct Chunk {
  size_t   begin = 0, end = 0;     // bytes of the stream: any garbage in front of the packet, then the packet
  Kind     kind = Kind::kJunk;
  uint32_t t_ms = 0;               // frames: the front-end's timestamp
  float    fs_hz = 0.0f;           // info packets: the announced frame rate
  double   due_ms = 0.0;           // device time at which the chunk is released
};

struct StreamStats {
  uint32_t frames = 0, infos = 0;
  bool     stamped = false;
};

// Cuts the stream into chunks that each end with one CRC-valid packet. A corrupted packet or garbage
// rides along with the next valid one (it arrived at about the same moment).
std::vector<Chunk> splitStream(const std::string& s, StreamStats& st) {
  std::vector<Chunk> out;
  rs::FrameParser fp;
  fp.reset();
  size_t begin = 0;
  for (size_t i = 0; i < s.size(); i++) {
    rs::FrameParser::Event ev = fp.push(static_cast<uint8_t>(s[i]));
    while (ev != rs::FrameParser::Event::kNone) {
      Chunk c;
      c.begin = begin;
      c.end = i + 1;
      begin = i + 1;                                   // events drained at the same byte carry no bytes of their own
      if (ev == rs::FrameParser::Event::kFrame) {
        c.kind = Kind::kFrame;
        c.t_ms = fp.frame().t_ms;
        st.frames++;
        if (c.t_ms != 0) st.stamped = true;
      } else {
        c.kind = Kind::kInfo;
        c.fs_hz = fp.info().frame_rate_hz;
        st.infos++;
      }
      out.push_back(c);
      ev = fp.poll();
    }
  }
  if (begin < s.size()) {                              // trailing bytes: an unfinished packet or garbage
    Chunk c;
    c.begin = begin;
    c.end = s.size();
    c.kind = Kind::kJunk;
    out.push_back(c);
  }
  return out;
}

// Release times. Stamped streams follow their own t_ms (a front-end restart, t_ms jumping back, is bridged
// by one frame period); unstamped ones are paced by sequence position and the announced frame rate.
void schedule(std::vector<Chunk>& cs, const StreamStats& st) {
  double last = 0.0, base = 0.0, fs = 20.0;
  uint32_t prev_t = 0, nframe = 0;
  for (size_t i = 0; i < cs.size(); i++) {
    Chunk& c = cs[i];
    if (c.kind == Kind::kInfo) {
      if (c.fs_hz > 0.5f && c.fs_hz < 1000.0f) fs = static_cast<double>(c.fs_hz);
      c.due_ms = last;
    } else if (c.kind == Kind::kFrame) {
      double due;
      if (st.stamped) {
        if (nframe > 0 && c.t_ms + 1000u < prev_t) base = last + 1000.0 / fs - static_cast<double>(c.t_ms);
        due = base + static_cast<double>(c.t_ms);
      } else {
        due = static_cast<double>(nframe) * 1000.0 / fs;
      }
      if (due < last) due = last;
      prev_t = c.t_ms;
      nframe++;
      last = due;
      c.due_ms = due;
    } else {
      c.due_ms = last;
    }
  }
}

// ---------------------------------------------------------------------------------------------- feeding
struct Item {
  uint64_t    due_us = 0;
  bool        console = false;     // a console line (otherwise bytes for the I/Q port)
  std::string bytes;
};

// Releases items in order at their due time, limited to each UART's byte rate (10 bit times per byte).
class Feeder {
 public:
  Feeder() {
    con_.port = &Serial;
    con_.per_ms = static_cast<double>(RS_SERIAL_BAUD) / 10000.0;
    iq_.port = &iqPort();
    iq_.per_ms = static_cast<double>(kIqBaud) / 10000.0;
  }
  void add(uint64_t due_us, bool console, const std::string& bytes) {
    Item it;
    it.due_us = due_us;
    it.console = console;
    it.bytes = bytes;
    items_.push_back(it);
  }
  void finalize() {
    std::stable_sort(items_.begin(), items_.end(), [](const Item& a, const Item& b) { return a.due_us < b.due_us; });
  }
  bool done() const { return items_.empty() && pos_ >= cur_.bytes.size(); }
  uint64_t sentBytes() const { return sent_; }

  void tick(uint64_t now_us) {
    refill(con_);
    if (!kIqOnConsole) refill(iq_);
    for (;;) {
      if (pos_ >= cur_.bytes.size()) {
        if (items_.empty() || items_.front().due_us > now_us) break;
        cur_ = items_.front();
        items_.pop_front();
        pos_ = 0;
        continue;
      }
      Lane& ln = (cur_.console || kIqOnConsole) ? con_ : iq_;
      const size_t room = static_cast<size_t>(ln.credit);
      if (room == 0) break;
      const size_t n = std::min(room, cur_.bytes.size() - pos_);
      ln.port->injectRx(reinterpret_cast<const uint8_t*>(cur_.bytes.data()) + pos_, n);
      pos_ += n;
      ln.credit -= static_cast<double>(n);
      sent_ += n;
    }
  }

 private:
  struct Lane {
    HardwareSerial* port = nullptr;
    double          per_ms = 0.0;
    double          credit = 0.0;
  };
  static void refill(Lane& l) {
    l.credit += l.per_ms;
    if (l.credit > l.per_ms + 1.0) l.credit = l.per_ms + 1.0;       // an idle line does not bank bytes
  }

  Lane con_, iq_;
  std::deque<Item> items_;
  Item   cur_;
  size_t pos_ = 0;
  uint64_t sent_ = 0;
};

// What the device printed: counts its scan records, keeps them, forwards everything.
struct DeviceOutput {
  bool        quiet = false;
  std::string line;
  std::vector<std::string> records;
  // Takes what the device has printed since the last call; returns it.
  std::string take() {
    const std::string out = Serial.tx();
    if (out.empty()) return out;
    Serial.clearTx();
    if (!quiet) {
      std::fwrite(out.data(), 1, out.size(), stdout);
      std::fflush(stdout);
    }
    for (size_t i = 0; i < out.size(); i++) {
      const char ch = out[i];
      if (ch == '\n') {
        if (!line.empty() && line[0] == '{' && line.find("\"verdict\"") != std::string::npos) records.push_back(line);
        line.clear();
      } else if (ch != '\r') {
        line.push_back(ch);
      }
    }
    return out;
  }
};

std::string jsonString(const std::string& json, const char* key) {
  const std::string k = std::string("\"") + key + "\":\"";
  const size_t p = json.find(k);
  if (p == std::string::npos) return "";
  const size_t b = p + k.size();
  const size_t e = json.find('"', b);
  return e == std::string::npos ? "" : json.substr(b, e - b);
}

bool readAll(const std::string& path, std::string& out) {
  out.clear();
  if (path == "-") {
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), stdin)) > 0) out.append(buf, n);
    return true;
  }
  std::ifstream f(path.c_str(), std::ios::binary);
  if (!f) return false;
  out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  return true;
}

struct Command {
  double      at_ms;
  std::string text;
};

struct Press {
  double  at_ms;
  uint8_t pin;
  uint32_t hold_ms;
};

struct Options {
  std::string stream_path;
  std::vector<Command> cmds;
  std::vector<Press>   presses;
  double      next_auto_cmd_ms = 500.0;
  uint32_t    until_results = 0;
  uint64_t    max_ms = 0;
  uint64_t    tail_ms = 4000;
  bool        screen = false;
  std::string dump_screen;
  bool        quiet = false;
  int         listen_port = -1;
  double      speed = 1.0;
};

void usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s --stream FILE|- [--cmd TEXT]... [--cmd-at MS TEXT]... [--press-at MS scan|mode|menu]...\n"
               "          [--long-press-at MS scan|mode|menu]... [--until-results N] [--max-ms MS] [--tail-ms MS]\n"
               "          [--screen] [--dump-screen FILE] [--quiet]\n"
               "       %s --listen PORT [--speed X] [--quiet]        (0 = pick a free port; UART0 build only)\n",
               argv0, argv0);
}

struct ButtonEvent {
  uint64_t at_us;
  uint8_t  pin;
  int      level;      // 0 = pressed (the buttons are wired to GND, internal pull-ups)
};

std::vector<ButtonEvent> buttonEvents(const std::vector<Press>& presses) {
  std::vector<ButtonEvent> ev;
  for (size_t i = 0; i < presses.size(); i++) {
    ButtonEvent down = {static_cast<uint64_t>(presses[i].at_ms * 1000.0), presses[i].pin, 0};
    ButtonEvent up = {down.at_us + static_cast<uint64_t>(presses[i].hold_ms) * 1000u, presses[i].pin, 1};
    ev.push_back(down);
    ev.push_back(up);
  }
  std::stable_sort(ev.begin(), ev.end(), [](const ButtonEvent& a, const ButtonEvent& b) { return a.at_us < b.at_us; });
  return ev;
}

bool parseButton(const char* name, uint8_t& pin) {
  if (!std::strcmp(name, "scan")) pin = PIN_BTN_SCAN;
  else if (!std::strcmp(name, "mode")) pin = PIN_BTN_MODE;
  else if (!std::strcmp(name, "menu")) pin = PIN_BTN_MENU;
  else return false;
  return true;
}

// ---------------------------------------------------------------------------------------------- offline run
int runOffline(const Options& o) {
  std::string stream;
  if (!readAll(o.stream_path, stream)) {
    std::fprintf(stderr, "cannot read %s\n", o.stream_path.c_str());
    return 1;
  }
  StreamStats st;
  std::vector<Chunk> chunks = splitStream(stream, st);
  schedule(chunks, st);
  if (st.frames == 0) {
    std::fprintf(stderr, "no valid frame packet in %s (%zu bytes)\n", o.stream_path.c_str(), stream.size());
    return 1;
  }

  Feeder feeder;
  double last_due = 0.0;
  for (size_t i = 0; i < chunks.size(); i++) {
    const Chunk& c = chunks[i];
    feeder.add(static_cast<uint64_t>(c.due_ms * 1000.0), false, stream.substr(c.begin, c.end - c.begin));
    last_due = std::max(last_due, c.due_ms);
  }
  for (size_t i = 0; i < o.cmds.size(); i++) {
    // a leading newline flushes whatever the console collected (binary noise before the line)
    feeder.add(static_cast<uint64_t>(o.cmds[i].at_ms * 1000.0), true, "\n" + o.cmds[i].text + "\n");
    last_due = std::max(last_due, o.cmds[i].at_ms);
  }
  feeder.finalize();

  stub::reset();
  stub::gfx().enabled = o.screen || !o.dump_screen.empty();
  Rig r;
  r.boot();
  if (!r.began) {
    std::fprintf(stderr, "firmware begin() failed\n");
    return 1;
  }
  r.tickers.push_back([&feeder](uint64_t t) { feeder.tick(t); });
  std::vector<ButtonEvent> buttons = buttonEvents(o.presses);
  size_t next_button = 0;
  r.tickers.push_back([&buttons, &next_button](uint64_t t) {
    while (next_button < buttons.size() && buttons[next_button].at_us <= t) {
      stub::setInput(buttons[next_button].pin, buttons[next_button].level);
      next_button++;
    }
  });
  for (size_t i = 0; i < o.presses.size(); i++) last_due = std::max(last_due, o.presses[i].at_ms + o.presses[i].hold_ms);
  DeviceOutput dev;
  dev.quiet = o.quiet;

  const uint64_t end_ms = o.max_ms ? o.max_ms : static_cast<uint64_t>(last_due) + o.tail_ms;
  uint64_t ms = 0, stop_at = 0;
  for (; ms < end_ms; ms++) {
    r.ms();
    dev.take();
    if (o.until_results && dev.records.size() >= o.until_results && stop_at == 0) stop_at = ms + 500;
    if (stop_at && ms >= stop_at) break;
  }
  r.run_ms(20);
  dev.take();

  std::fprintf(stderr, "replay: %u frame packets (%u info) from %zu bytes, %s; ran %.1f s of device time, %zu scan record(s)\n",
               st.frames, st.infos, stream.size(), st.stamped ? "timestamped" : "unstamped (paced by frame rate)",
               static_cast<double>(ms) / 1000.0, dev.records.size());
  for (size_t i = 0; i < dev.records.size(); i++)
    std::fprintf(stderr, "  scan %s: %s (%s) evidence=%s\n", jsonString(dev.records[i], "scan_id").c_str(),
                 jsonString(dev.records[i], "verdict").c_str(), jsonString(dev.records[i], "invalid_reason").c_str(),
                 jsonString(dev.records[i], "evidence").c_str());
  if (o.screen) std::fprintf(stderr, "screen: %s\n", r.screen().c_str());
  if (!o.dump_screen.empty() && !stub::gfx().dump(o.dump_screen)) std::fprintf(stderr, "cannot write %s\n", o.dump_screen.c_str());
  if (o.until_results && dev.records.size() < o.until_results) {
    std::fprintf(stderr, "expected %u scan record(s), got %zu\n", o.until_results, dev.records.size());
    return 3;
  }
  return 0;
}

// ---------------------------------------------------------------------------------------------- online run
#if RS_REPLAY_SOCKETS
bool sendAll(int fd, const char* p, size_t n) {
  while (n > 0) {
    const ssize_t w = send(fd, p, n, MSG_NOSIGNAL);
    if (w > 0) {
      p += w;
      n -= static_cast<size_t>(w);
      continue;
    }
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      struct pollfd pf;
      pf.fd = fd;
      pf.events = POLLOUT;
      pf.revents = 0;
      if (poll(&pf, 1, 200) <= 0) return false;
      continue;
    }
    return false;
  }
  return true;
}

int runOnline(const Options& o) {
#if !RS_IQ_ON_UART0
  (void)o;
  std::fprintf(stderr, "--listen needs the UART0 build (-DRS_IQ_ON_UART0=1): the stream and the console share one port\n");
  return 2;
#else
  signal(SIGPIPE, SIG_IGN);
  const int ls = socket(AF_INET, SOCK_STREAM, 0);
  if (ls < 0) {
    std::perror("socket");
    return 1;
  }
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in a;
  std::memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);              // loopback only: this is a test device, not a service
  a.sin_port = htons(static_cast<uint16_t>(o.listen_port));
  if (bind(ls, reinterpret_cast<struct sockaddr*>(&a), sizeof(a)) < 0 || listen(ls, 1) < 0) {
    std::perror("bind/listen");
    close(ls);
    return 1;
  }
  socklen_t alen = sizeof(a);
  getsockname(ls, reinterpret_cast<struct sockaddr*>(&a), &alen);
  std::fprintf(stderr, "listening on 127.0.0.1:%d\n", ntohs(a.sin_port));
  std::fflush(stderr);
  const int fd = accept(ls, nullptr, nullptr);
  close(ls);
  if (fd < 0) {
    std::perror("accept");
    return 1;
  }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

  stub::reset();
  stub::gfx().enabled = false;
  Rig r;
  r.boot();
  if (!r.began) {
    std::fprintf(stderr, "firmware begin() failed\n");
    close(fd);
    return 1;
  }
  // bytes from the client reach the port at the UART's rate
  std::string rxq;
  size_t rxpos = 0;
  double credit = 0.0;
  const double per_ms = static_cast<double>(RS_SERIAL_BAUD) / 10000.0;
  r.tickers.push_back([&](uint64_t) {
    credit += per_ms;
    if (credit > per_ms + 1.0) credit = per_ms + 1.0;
    const size_t room = static_cast<size_t>(credit);
    const size_t n = std::min(room, rxq.size() - rxpos);
    if (n > 0) {
      Serial.injectRx(reinterpret_cast<const uint8_t*>(rxq.data()) + rxpos, n);
      rxpos += n;
      credit -= static_cast<double>(n);
      if (rxpos == rxq.size()) {
        rxq.clear();
        rxpos = 0;
      }
    }
  });

  DeviceOutput dev;
  dev.quiet = o.quiet;
  const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  uint64_t vms = 0;
  bool open = true, send_ok = true;
  auto flushOut = [&]() {
    const std::string out = dev.take();
    if (!out.empty() && send_ok && !sendAll(fd, out.data(), out.size())) send_ok = false;
  };
  while (open && send_ok) {
    char buf[4096];
    for (;;) {
      const ssize_t n = recv(fd, buf, sizeof(buf), 0);
      if (n > 0) {
        rxq.append(buf, static_cast<size_t>(n));
        continue;
      }
      if (n == 0) open = false;
      else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) open = false;
      break;
    }
    const double wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const uint64_t target = static_cast<uint64_t>(wall_ms * o.speed);
    while (vms < target) {
      r.ms();
      vms++;
      if ((vms % 5) == 0) flushOut();
    }
    usleep(300);
  }
  r.run_ms(50);                                           // whatever the device still had to say
  flushOut();
  close(fd);
  std::fprintf(stderr, "session ended after %.1f s of device time, %zu scan record(s)\n", static_cast<double>(vms) / 1000.0, dev.records.size());
  return 0;
#endif
}
#endif  // RS_REPLAY_SOCKETS

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    const bool more = i + 1 < argc;
    if (a == "--stream" && more) o.stream_path = argv[++i];
    else if (a == "--cmd" && more) {
      Command c;
      c.at_ms = o.next_auto_cmd_ms;
      c.text = argv[++i];
      o.next_auto_cmd_ms += 2000.0;
      o.cmds.push_back(c);
    } else if (a == "--cmd-at" && i + 2 < argc) {
      Command c;
      c.at_ms = std::atof(argv[++i]);
      c.text = argv[++i];
      o.next_auto_cmd_ms = c.at_ms + 2000.0;
      o.cmds.push_back(c);
    } else if ((a == "--press-at" || a == "--long-press-at") && i + 2 < argc) {
      Press p;
      p.at_ms = std::atof(argv[++i]);
      p.hold_ms = a == "--press-at" ? 90u : 1100u;
      if (!parseButton(argv[++i], p.pin)) {
        usage(argv[0]);
        return 2;
      }
      o.presses.push_back(p);
    } else if (a == "--until-results" && more) o.until_results = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    else if (a == "--max-ms" && more) o.max_ms = std::strtoull(argv[++i], nullptr, 10);
    else if (a == "--tail-ms" && more) o.tail_ms = std::strtoull(argv[++i], nullptr, 10);
    else if (a == "--screen") o.screen = true;
    else if (a == "--dump-screen" && more) o.dump_screen = argv[++i];
    else if (a == "--quiet") o.quiet = true;
    else if (a == "--listen" && more) o.listen_port = std::atoi(argv[++i]);
    else if (a == "--speed" && more) o.speed = std::atof(argv[++i]);
    else {
      usage(argv[0]);
      return 2;
    }
  }
  if (o.listen_port >= 0) {
    if (o.speed <= 0.0 || !o.stream_path.empty()) {
      usage(argv[0]);
      return 2;
    }
#if RS_REPLAY_SOCKETS
    return runOnline(o);
#else
    std::fprintf(stderr, "--listen is not available on this platform\n");
    return 2;
#endif
  }
  if (o.stream_path.empty()) {
    usage(argv[0]);
    return 2;
  }
  return runOffline(o);
}
