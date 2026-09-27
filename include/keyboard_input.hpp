#include <iostream>
#include <termios.h>
#include <unistd.h>

struct termios orig_termios;

inline void disable_raw_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

inline void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO); // no line buffering, no echo
    raw.c_cc[VMIN] = 0;   // non-blocking read
    raw.c_cc[VTIME] = 0;  // no timeout wait
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    atexit(disable_raw_mode); // restore terminal on exit
}

#define UP_ARROW 1000
#define DOWN_ARROW 1001

inline int read_key() {
    char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) return 0; // nothing pressed

    if (c == '\x1b') { // possible escape sequence
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) <= 0) return '\x1b';
        if (read(STDIN_FILENO, &seq[1], 1) <= 0) return '\x1b';
        if (seq[0] == '[') {
            if (seq[1] == 'A') return 1000; // up
            if (seq[1] == 'B') return 1001; // down
        }
        return '\x1b';
    }
    return c;
}