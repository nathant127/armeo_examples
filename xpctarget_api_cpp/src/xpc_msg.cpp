#include "xpc_msg.hpp"
#include <iostream>

#define DO_BE_CONV 1

constexpr uint8_t MAGIC_NUM1 = 0x4e;
constexpr uint8_t MAGIC_NUM2 = 0xcb;

Message::Message() {

}

Message::~Message() {

}

template <typename DataType>
void Message::push(const std::vector<DataType> buf) {
    push(buf.data(), buf.size());
}

template <typename DataType>
void Message::push(const DataType* buf, size_t buf_size) {
    size_t buf_byte_size = buf_size * sizeof(DataType);
    Fmt next_fmt = get_next_fmt();
    size_t expected_size = fmt_size(next_fmt);

    if (buf_size == 0) {
        throw std::logic_error("Tried to push empty buffer");
    }

    if (expected_size == 0) { 
        throw std::logic_error("tried to push when payload is done");
    }
    if (expected_size != -1 && // If buffer is not an array
            expected_size != buf_byte_size) // If the size of the given buffer is not equal to the expected size
    {
        std::ostringstream emsg;
        emsg << "tried to push buffer with incorrect size: " << buf_byte_size << ", expected " << expected_size;
        throw std::logic_error(emsg.str());
    }

    // Add the length prefix if data is an array
    if (expected_size == -1) {
        int len_prefix = buf_size;

        payload.resize(payload.size() + sizeof(len_prefix));
        std::byte* p_write = &payload[payload.size() - sizeof(len_prefix)];

        memcpy(p_write, &len_prefix, sizeof(len_prefix));
        *reinterpret_cast<int*>(&payload[4]) += sizeof(len_prefix);
    }

    // Would this push exceed the first payloads max size?
    if (!is_payload1_full && (payload.size() + buf_byte_size) > payload1_maxsize) {
        is_payload1_full = true;
        payload1_finalsize = payload.size();
    }

    // Resize and get the write position of the vector
    payload.resize(payload.size() + buf_byte_size);
    std::byte* p_write = &payload[payload.size() - buf_byte_size];
    // Copy buffer into payload
    memcpy(p_write, buf, buf_byte_size);
    // Increment header size count
    *reinterpret_cast<int*>(&payload[4]) += buf_byte_size;
}

template void Message::push<uint8_t>(const uint8_t* buf, size_t buf_size);
template void Message::push<char>(const char* buf, size_t buf_size);
template void Message::push<uint16_t>(const uint16_t* buf, size_t buf_size);
template void Message::push<int16_t>(const int16_t* buf, size_t buf_size);
template void Message::push<uint32_t>(const uint32_t* buf, size_t buf_size);
template void Message::push<int32_t>(const int32_t* buf, size_t buf_size);
template void Message::push<double>(const double* buf, size_t buf_size);

void Message::push_str(const char* buf) {
    int n = std::strlen(buf);
    push<char>(buf, n+1);
}

int Message::send(const Connection& con) {
    read_idx = 0;

    void* p1;
    void* p2;
    size_t p1_len, p2_len;

    get_tx_payloads(p1, p1_len, p2, p2_len);

    if (con.con_send(p1, p1_len) == -1)
    {
        std::runtime_error("Failed to send msg");
    }
    if (p2_len != 0 && 
        con.con_send(p2, p2_len) == -1) 
    {
        std::runtime_error("Failed to send msg p2");
    }


    if (con.con_recv(&res_header, sizeof(res_header)) != sizeof(res_header)) {
        std::runtime_error("Failed to read response header");
    }

    if (res_header.size <= 0) {
        return 0;
    }

    res_data.resize(res_header.size);
    if (con.con_recv(res_data.data(), res_header.size) != res_header.size) {
        std::runtime_error("Failed to read response data");
    }

    
    return 0;
}

template <typename Type>
void Message::read(Type& x) {
    memcpy(&x, &res_data[read_idx], sizeof(x));
    read_idx += sizeof(x);
}

template void Message::read<uint8_t>(uint8_t& x);
template void Message::read<char>(char& x);
template void Message::read<uint16_t>(uint16_t& x);
template void Message::read<int16_t>(int16_t& x);
template void Message::read<uint32_t>(uint32_t& x);
template void Message::read<int32_t>(int32_t& x);
template void Message::read<double>(double& x);

template<typename Type>
void Message::read(std::vector<Type>& vec) {
    vec.clear();

    int32_t size;
    memcpy(&size, &res_data[read_idx], sizeof(size));
    read_idx += sizeof(size);

    int32_t bytesize = size * sizeof(Type);

    vec.resize(size);
    memcpy(vec.data(), &res_data[read_idx], bytesize);
    read_idx += bytesize;
}

template void Message::read<uint32_t>(std::vector<uint32_t>& x);
template void Message::read<int32_t>(std::vector<int32_t>& x);
template void Message::read<uint16_t>(std::vector<uint16_t>& x);
template void Message::read<int16_t>(std::vector<int16_t>& x);
template void Message::read<uint8_t>(std::vector<uint8_t>& x);
template void Message::read<char>(std::vector<char>& x);
template void Message::read<double>(std::vector<double>& x);


template<typename Type>
void Message::read(Type* arr) {

    int32_t size;
    memcpy(&size, &res_data[read_idx], sizeof(size));
    read_idx += sizeof(size);

    int32_t bytesize = size * sizeof(Type);

    memcpy(arr, &res_data[read_idx], bytesize);
    read_idx += bytesize;
}

template void Message::read<uint32_t>(uint32_t* x);
template void Message::read<int32_t>(int32_t* x);
template void Message::read<uint16_t>(uint16_t* x);
template void Message::read<int16_t>(int16_t* x);
template void Message::read<uint8_t>(uint8_t* x);
template void Message::read<char>(char* x);
template void Message::read<double>(double* x);

void Message::read(std::string& str) {
    str.clear();

    int32_t size;
    memcpy(&size, &res_data[read_idx], sizeof(size));
    read_idx += sizeof(size);

    str.assign(reinterpret_cast<const char*> (&res_data[read_idx]));
    read_idx += size;
}

void Message::get_tx_payloads(void*& p1, size_t& p1len, void*& p2, size_t& p2len) {
    if (!is_payload1_full) {
        p1 = payload.data();
        p1len = payload.size();
        p2 = nullptr;
        p2len = 0;
    }
    else {
        p1 = payload.data();
        p1len = payload1_finalsize;
        p2 = payload.data() + payload1_finalsize;
        p2len = payload.size() - payload1_finalsize;
    }

    return;
}


void Message::reset(uint8_t cmd, uint8_t subcmd) {
    cmd_ = cmd;
    subcmd_ = subcmd;
    payload.resize(8, std::byte{0x00});
    payload[0] = static_cast<std::byte>(MAGIC_NUM1);
    payload[1] = static_cast<std::byte>(MAGIC_NUM2);
    payload[2] = static_cast<std::byte>(cmd);
    payload[3] = static_cast<std::byte>(subcmd);
    *reinterpret_cast<int*>(&payload[4]) = 0;

    is_payload1_full = false;
    payload1_finalsize = 0;

    res_data.clear();
    auto& tx_fmt = cmd_table.at(Cmd{cmd, subcmd}).tx_fmt;
    tx_fmt_vec_it_ = tx_fmt.begin();
    tx_fmt_vec_end_ = tx_fmt.end();
}

Fmt Message::get_next_fmt() {
    Fmt return_fmt = Fmt::NONE;
    // If at end of fmt table, just return none and don't increment
    if (tx_fmt_vec_it_ == tx_fmt_vec_end_)
        return Fmt::NONE;

    return_fmt = *tx_fmt_vec_it_;
    tx_fmt_vec_it_++;
    
    return return_fmt;
}
