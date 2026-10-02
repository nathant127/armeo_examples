#pragma once

#include "xpc_comms.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <unordered_map>
#include <sstream>

enum class Fmt : uint8_t;

class Message {
public:
    Message();
    ~Message();



    template <typename DataType>
    void push(const DataType* buf, size_t buf_size);
    
    template <typename DataType>
    void push(const std::vector<DataType> buf);

    void push_str(const char* buf);

    int send(const Connection& con);

    template <typename Type>
    void read(Type& x);
    template<typename Type>
    void read(std::vector<Type>& vec);
    template<typename Type>
    void read(Type* arr);
    void read(std::string& str);

    
    void reset(uint8_t cmd, uint8_t subcmd);
    
    
    private:
    Fmt get_next_fmt();
    void get_tx_payloads(void*& p1, size_t& p1len, void*& p2, size_t& p2len);
    
    private:
    const size_t payload1_maxsize = 112;
    std::vector<std::byte> payload;
    bool is_payload1_full = false;
    size_t payload1_finalsize;
    
    uint8_t cmd_;
    uint8_t subcmd_;
    std::vector<Fmt>::iterator tx_fmt_vec_it_;
    std::vector<Fmt>::iterator tx_fmt_vec_end_;

    struct {
        uint8_t magic_num1;
        uint8_t magic_num2;
        uint8_t cmd;
        uint8_t subcmd;
        int size;

    } res_header;
    std::vector<std::byte> res_data;
    size_t read_idx;
    
};



enum class Fmt : uint8_t {
    NONE,
    CHAR,
    BYTE,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
    STR,
    ARR_BYTE,
    ARR_I16,
    ARR_U16,
    ARR_I32,
    ARR_U32,
    ARR_I64,
    ARR_U64,
    ARR_F32,
    ARR_F64,
};

inline bool is_fmt_dyn(Fmt fmt) {
    switch (fmt){
        case Fmt::STR:
        case Fmt::ARR_BYTE:
        case Fmt::ARR_I16:
        case Fmt::ARR_U16:
        case Fmt::ARR_I32:
        case Fmt::ARR_U32:
        case Fmt::ARR_I64:
        case Fmt::ARR_U64:
        case Fmt::ARR_F32:
        case Fmt::ARR_F64:
            return true;
        default:
            return false;
    }
}

inline size_t fmt_size(Fmt fmt) {
    switch (fmt){
        case Fmt::CHAR:
        case Fmt::BYTE:
            return 1;
        case Fmt::I16:
        case Fmt::U16:
            return 2;
        case Fmt::I32:
        case Fmt::U32:
        case Fmt::F32:
            return 4;
        case Fmt::I64:
        case Fmt::U64:
        case Fmt::F64:
            return 8;
        case Fmt::STR:
        case Fmt::ARR_BYTE:
        case Fmt::ARR_I16:
        case Fmt::ARR_U16:
        case Fmt::ARR_I32:
        case Fmt::ARR_U32:
        case Fmt::ARR_I64:
        case Fmt::ARR_U64:
        case Fmt::ARR_F32:
        case Fmt::ARR_F64:
            return -1;
        case Fmt::NONE:
        default:
            return 0;
    }
}





// Cmd Table is at 18001bb60

/*
    'u<X>' -> Unsigned X
    'X*' -> array of X prefixed by length
    Format Code         Size    Assumed Type
    'F' = 0x46 | 70     ?       ?
    'b' = 0x0B | 11     1       uint8_t
    'c' = 0x0A | 10     1       char
    'd' = 0x1E | 30     4       int32_t | uint32_t
    'f' = 0x28 | 40     8       double
    'i' = 0x14 | 20     2       int16_t | uint16_t
    's' = 0x3C | 60     1       cstring?
    '\0' = 0xC8 | 200   0       None?
*/

struct Cmd {
    uint8_t cmd;
    uint8_t subcmd;
    bool operator==(const Cmd& other) const {
        return cmd == other.cmd && subcmd == other.subcmd;
    }
};

namespace std {
    template <>
    struct hash<Cmd> {
        size_t operator()(const Cmd& c) const noexcept {
            // pack both bytes into one value and hash that
            return std::hash<uint16_t>()(
                (static_cast<uint16_t>(c.cmd) << 8) | c.subcmd
            );
        }
    };
}

struct CmdInfo {
    std::vector<Fmt> tx_fmt;
    std::vector<Fmt> rx_fmt;
};

inline std::unordered_map<Cmd, CmdInfo> cmd_table = {
    {{0xC8, 0x0A}, {{}, {Fmt::STR}}},
    {{0xC8, 0x14}, {{}, {Fmt::BYTE}}},
    {{0xC8, 0x1E}, {{Fmt::BYTE}, {}}},
    {{0xC8, 0x28}, {{}, {Fmt::BYTE}}},
    {{0xC8, 0x32}, {{Fmt::BYTE}, {}}},
    {{0xC8, 0x3C}, {{}, {Fmt::STR}}},
    {{0xC8, 0x46}, {{}, {}}},
    {{0xC8, 0x50}, {{}, {Fmt::ARR_BYTE}}},
    {{0xC8, 0x64}, {{Fmt::I32, Fmt::STR}, {Fmt::ARR_BYTE}}},
    {{0xC8, 0x8C}, {{Fmt::ARR_BYTE}, {}}},
    {{0xC8, 0x96}, {{}, {Fmt::F64}}},
    {{0xC8, 0xA0}, {{}, {Fmt::I32}}},
    {{0xC8, 0xAA}, {{Fmt::I32}, {}}},

    {{0x28, 0x0A}, {{Fmt::ARR_U32}, {Fmt::ARR_F64}}},
    {{0x28, 0x14}, {{Fmt::U32}, {Fmt::ARR_F64}}},
    {{0x28, 0x1E}, {{Fmt::U32, Fmt::ARR_F64}, {}}},
    {{0x28, 0x32}, {{Fmt::STR}, {}}},
    {{0x28, 0x3C}, {{Fmt::STR}, {}}},
    {{0x28, 0x46}, {{Fmt::STR}, {Fmt::I32}}},
    {{0x28, 0x50}, {{Fmt::U32}, {Fmt::STR}}},
    {{0x28, 0x5A}, {{Fmt::STR}, {Fmt::I32, Fmt::I32}}},
    {{0x28, 0x64}, {{Fmt::U32}, {Fmt::STR}}},
    {{0x28, 0x6E}, {{Fmt::U32}, {Fmt::I32}}},
    {{0x28, 0x78}, {{Fmt::U32}, {Fmt::ARR_I32}}},
    {{0x28, 0x82}, {{Fmt::STR, Fmt::STR}, {Fmt::U32}}},
    {{0x28, 0x8C}, {{Fmt::U32}, {Fmt::STR, Fmt::STR}}},
    {{0x28, 0x96}, {{Fmt::U32}, {Fmt::STR}}},
    {{0x28, 0xA0}, {{Fmt::U32}, {Fmt::U32}}},

    {{0x14, 0x0A}, {{}, {Fmt::F64}}},
    {{0x14, 0x14}, {{Fmt::F64}, {}}},
    {{0x14, 0x1E}, {{}, {Fmt::F64}}},
    {{0x14, 0x28}, {{}, {Fmt::F64}}},
    {{0x14, 0x32}, {{Fmt::F64}, {}}},
    {{0x14, 0x6E}, {{}, {Fmt::F64, Fmt::F64}}},
    {{0x14, 0x78}, {{}, {Fmt::F64, Fmt::F64}}},
    {{0x14, 0x82}, {{}, {Fmt::F64}}},
    {{0x14, 0x8C}, {{Fmt::U32, Fmt::U32, Fmt::U32}, {Fmt::ARR_BYTE}}},
    {{0x14, 0x96}, {{}, {Fmt::U32}}},
    {{0x14, 0x97}, {{}, {Fmt::U32}}},
    {{0x14, 0x9B}, {{Fmt::U32}, {}}},
    {{0x14, 0x9C}, {{}, {Fmt::U32}}},
    {{0x14, 0x9E}, {{}, {Fmt::U32}}},
    {{0x14, 0xA1}, {{}, {Fmt::U32}}},
    {{0x14, 0xA2}, {{}, {Fmt::U32}}},
    {{0x14, 0xA3}, {{}, {Fmt::U32}}},
    {{0x14, 0xA4}, {{}, {Fmt::U32}}},
    {{0x14, 0xA5}, {{}, {Fmt::U32}}},
    {{0x14, 0xA6}, {{}, {Fmt::U32}}},
    {{0x14, 0xA7}, {{}, {}}},
    {{0x14, 0xA8}, {{}, {Fmt::U32}}},
    {{0x14, 0xAC}, {{}, {}}},

    {{0x1E, 0x0A}, {{Fmt::I32, Fmt::I32, Fmt::I32, Fmt::I32, Fmt::I32}, {Fmt::ARR_F64}}},
    {{0x1E, 0x0F}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x10}, {{Fmt::U32}, {}}},
    {{0x1E, 0x14}, {{Fmt::U32}, {Fmt::F64}}},
    {{0x1E, 0x19}, {{Fmt::U32}, {Fmt::F64}}},
    {{0x1E, 0x1E}, {{Fmt::U32, Fmt::F64}, {}}},
    {{0x1E, 0x23}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x28}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x2D}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x32}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x37}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x3C}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x41}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x46}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x4B}, {{Fmt::U32}, {Fmt::I32}}},
    {{0x1E, 0x50}, {{Fmt::U32, Fmt::I32}, {}}},
    {{0x1E, 0x55}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x5A}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x64}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x65}, {{Fmt::U32, Fmt::I32}, {}}},
    {{0x1E, 0x66}, {{Fmt::U32}, {Fmt::I32}}},
    {{0x1E, 0x68}, {{Fmt::U32, Fmt::I32}, {}}},
    {{0x1E, 0x6A}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x6C}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x6E}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x78}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x79}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x7D}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x7E}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x7F}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x80}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x81}, {{Fmt::U32, Fmt::U32}, {}}},
    {{0x1E, 0x82}, {{Fmt::U32}, {Fmt::F64, Fmt::F64}}},
    {{0x1E, 0x83}, {{Fmt::U32, Fmt::F64, Fmt::F64}, {}}},
    {{0x1E, 0x84}, {{}, {Fmt::U32}}},
    {{0x1E, 0x85}, {{Fmt::U32}, {}}},
    {{0x1E, 0x87}, {{Fmt::U32, Fmt::U32}, {Fmt::STR}}},
    {{0x1E, 0x8C}, {{Fmt::U32, Fmt::U32, Fmt::STR}, {}}},
    {{0x1E, 0x91}, {{Fmt::U32}, {Fmt::U32}}},
    {{0x1E, 0x96}, {{Fmt::U32}, {}}},
    {{0x1E, 0x97}, {{Fmt::U32}, {}}},
    {{0x1E, 0x98}, {{Fmt::U32}, {}}},
    {{0x1E, 0x99}, {{}, {Fmt::U32}}},
    {{0x1E, 0x9A}, {{}, {Fmt::U32}}},
    {{0x1E, 0x9B}, {{}, {Fmt::ARR_I32}}},
    {{0x1E, 0x9C}, {{}, {Fmt::ARR_I32}}},
    {{0x1E, 0x9D}, {{}, {Fmt::ARR_I32}}},
    {{0x1E, 0x9E}, {{}, {Fmt::ARR_I32}}},
    {{0x1E, 0x9F}, {{Fmt::U32}, {Fmt::ARR_I32}}},
    {{0x1E, 0xA0}, {{Fmt::U32}, {Fmt::ARR_I32}}},
    {{0x1E, 0xA1},
     {{Fmt::U32},
      {Fmt::F64,  Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32,
       Fmt::U32,  Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32,
       Fmt::U32,  Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32,
       Fmt::U32}}},
    {{0x1E, 0xA2},
     {{Fmt::U32,  Fmt::F64, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32,
       Fmt::U32,  Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32,
       Fmt::U32,  Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32, Fmt::U32},
      {}}},
};