--[[
xpc.lua - Wireshark Lua dissector for the MathWorks xpcapi (xPC Target)
TCP protocol.

Header (8 bytes):
  0-1   magic     uint16
  2     cmd       uint8
  3     subcmd    uint8
  4-7   length    uint32   (length of the DATA PAYLOAD only, not including
                            the 8-byte header)

Payload:
  Sequence of up to 5 fields, whose types depend on (cmd,subcmd) AND on
  whether this message is a request (client -> target) or a response
  (target -> client). See CMD_TABLE below, built from your C++ cmd_table.

  Arrays and strings are assumed length-prefixed: a 4-byte uint32 count,
  followed by that many elements/bytes. If xpcapi actually encodes strings
  differently (e.g. null-terminated), adjust parse_field's STR branch.

INSTALL:
  Copy this file into Wireshark's personal Lua plugins folder:
    Help -> About Wireshark -> Folders -> "Personal Lua Plugins"
  Then restart Wireshark, or press Ctrl+Shift+L to reload.

REQUEST/RESPONSE DIRECTION:
  Wireshark can't tell request from response by looking at bytes alone --
  it depends on which side sent the packet. This dissector decides using
  XPC_SERVER_PORT below: traffic TO that port is a request, traffic FROM
  it is a response. xPC Target's default target-communication port is
  22222 -- confirm/adjust for your setup.
]]

----------------------------------------------------------------------
-- CONFIG
----------------------------------------------------------------------

local MAGIC = 0xCB4E          -- confirmed from capture (raw bytes 4e cb, little-endian)

-- "le" = little-endian (typical Winsock/x86 default), "be" = big-endian.
-- Applies uniformly to header and payload multi-byte fields.
local ENDIANNESS = "le"

-- TCP port the xPC target listens on. Used to determine request vs.
-- response direction. Default 22222 is xPC Target's traditional default --
-- verify against your actual target/host setup.
local XPC_SERVER_PORT = 22222

-- Safety cap on the payload "length" field. If a packet's magic bytes
-- match by coincidence (or the length field is misparsed due to wrong
-- ENDIANNESS/offset), a garbage length value can cause Wireshark to
-- request a massive TCP reassembly (desegment_len in the billions),
-- which can crash or hang some Wireshark versions. Anything above this
-- is treated as "not actually our protocol" instead of being acted on.
local MAX_PAYLOAD_LEN = 1024 * 1024  -- 1 MB; adjust if your payloads are legitimately larger

----------------------------------------------------------------------
-- Fmt -> internal type mapping
-- (mirrors your C++ Fmt enum; extend as you add more formats)
----------------------------------------------------------------------

local FMT_MAP = {
    NONE     = nil,                                  -- marks end of field list
    BYTE     = { type = "uint8" },
    I8       = { type = "int8" },
    U16      = { type = "uint16" },
    I16      = { type = "int16" },
    U32      = { type = "uint32" },
    I32      = { type = "int32" },
    U64      = { type = "uint64" },
    I64      = { type = "int64" },
    F32      = { type = "float" },
    F64      = { type = "double" },
    STR      = { type = "string" },
    ARR_BYTE = { type = "array", elem = "uint8" },
    ARR_U32  = { type = "array", elem = "uint32" },
    ARR_I32  = { type = "array", elem = "int32" },
    ARR_F64  = { type = "array", elem = "double" },
    ARR_F32  = { type = "array", elem = "float" },
}

-- Builds a field-spec list from a compact array of Fmt-name strings,
-- e.g. {"ARR_U32"} or {"STR", "I32"}. Stops at the first "NONE"/nil or
-- once 5 entries are consumed (matching your CmdInfo fixed size).
local function build_fields(fmt_names)
    local fields = {}
    for i, fname in ipairs(fmt_names or {}) do
        if i > 5 then break end
        local m = FMT_MAP[fname]
        if m == nil then break end -- NONE or unrecognized -> stop
        fields[#fields + 1] = {
            name = string.format("arg%d_%s", i, fname),
            type = m.type,
            elem = m.elem,
        }
    end
    return fields
end

----------------------------------------------------------------------
-- CMD_TABLE -- transcribed from your C++ cmd_table
----------------------------------------------------------------------

local CMD_TABLE = {}

local function register_cmd(cmd, subcmd, name, req_fmts, resp_fmts)
    CMD_TABLE[cmd] = CMD_TABLE[cmd] or {}
    CMD_TABLE[cmd][subcmd] = {
        name = name,
        req  = build_fields(req_fmts),
        resp = build_fields(resp_fmts),
    }
end

local function repeat_fmt(fmt, count)
    local result = {}
    for i = 1, count do
        table.insert(result, fmt)
    end
    return result
end

--                cmd   subcmd  name                    request fmts     response fmts

-- All 0xC8 cmds:
register_cmd(0xC8, 0x0A, "GET_TARGET_VERSION",        {  },              { "STR" }         )
register_cmd(0xC8, 0x14, "GET_ECHO",        {  },              { "BYTE" }         )
register_cmd(0xC8, 0x1E, "SET_ECHO",        { "BYTE" },              {  }         )
register_cmd(0xC8, 0x28, "GET_HIDDEN_SCOPE_ECHO",        {  },              { "BYTE" }         )
register_cmd(0xC8, 0x32, "SET_HIDDEN_SCOPE_ECHO",        { "BYTE" },              {  }         )
register_cmd(0xC8, 0x3C, "CHECK_LOADED",            {},                     { "STR" }         )
register_cmd(0xC8, 0x46, "REBOOT",        {  },              {  }         )
register_cmd(0xC8, 0x50, "GET_PCI_INFO",        {  },              { "ARR_BYTE" }         )
register_cmd(0xC8, 0x64, "UNKOWN_PURPOSE",        { "I32", "STR" },              { "ARR_BYTE" }         )
register_cmd(0xC8, 0x8C, "LOAD_APP",                { "ARR_BYTE" },         {}                )
register_cmd(0xC8, 0x96, "GET_SESSION_TIME",        {  },              { "F64" }         )
register_cmd(0xC8, 0xA0, "IS_TARGET_SCOPE",        {  },              { "I32" }         )
register_cmd(0xC8, 0xAA, "SET_TARGET_SCOPE_UPDATE",        { "I32" },              {  }         )

-- All 0x28 cmds:
register_cmd(0x28, 0x0A, "SIG_GET",                 { "ARR_U32" },          { "ARR_F64" }     )
register_cmd(0x28, 0x14, "PAR_GET",                 { "U32" },              { "ARR_F64" }         )
register_cmd(0x28, 0x1E, "PAR_SET",                 { "U32", "ARR_F64" },   { }         )
register_cmd(0x28, 0x32, "SAVE_PARAM_SET",        { "STR" },              {  }         )
register_cmd(0x28, 0x3C, "LOAD_PARAM_SET",        { "STR" },              {  }         )
register_cmd(0x28, 0x46, "SIG_GET_ID",              { "STR" },              { "I32" }         )
register_cmd(0x28, 0x50, "SIG_GET_NAME",            { "U32" },              { "STR" }         )
register_cmd(0x28, 0x5A, "SIG_GET_LABEL_IDS",        { "STR" },              { "I32", "I32" }         )
register_cmd(0x28, 0x64, "SIG_GET_LABEL",           { "U32" },              { "STR" }         )
register_cmd(0x28, 0x6E, "PAR_GET_DIM_SIZE",        { "U32" },              { "I32" }         )
register_cmd(0x28, 0x78, "PAR_GET_DIMS",               { "U32" },              { "ARR_I32" }         )
register_cmd(0x28, 0x82, "PAR_GET_ID",              { "STR", "STR" },       { "U32" }         )
register_cmd(0x28, 0x8C, "PAR_GET_NAME",            { "U32" },              { "STR", "STR" }         )
register_cmd(0x28, 0x96, "PAR_GET_TYPE",            { "U32" },              { "STR" }         )
register_cmd(0x28, 0xA0, "SIG_GET_WIDTH",           { "U32" },              { "U32" }         )

-- All 0x14 cmds:
register_cmd(0x14, 0x0A, "GET_STOP_TIME",           {},                     { "F64" }         )
register_cmd(0x14, 0x14, "SET_STOP_TIME",        { "F64" },              {  }         )
register_cmd(0x14, 0x1E, "GET_EXEC_TIME",        {  },              { "F64" }         )
register_cmd(0x14, 0x28, "GET_SAMPLE_TIME",         {},                     { "F64" }         )
register_cmd(0x14, 0x32, "SET_SAMPLE_TIME",        { "F64" },              {  }         )
register_cmd(0x14, 0x6E, "MAXIMUM_TET",        {  },              { "F64", "F64" }         )
register_cmd(0x14, 0x78, "MINIMUM_TET",        { },              { "F64", "F64" }         )
register_cmd(0x14, 0x82, "AVERAGE_TET",        {  },              { "F64" }         )
register_cmd(0x14, 0x8C, "READ_XML",        { "U32", "U32", "U32" },              { "ARR_BYTE" }         )
register_cmd(0x14, 0x96, "GET_PAR_NUM",                 {},                     { "U32" })
register_cmd(0x14, 0x97, "GET_SIG_NUM",                 {},                     { "U32" }) 
register_cmd(0x14, 0x9B, "START/STOP",              { "U32" },              {}                )
register_cmd(0x14, 0x9C, "GET_NUM_STATES",        {  },              { "U32" }         )
register_cmd(0x14, 0x9E, "GET_NUM_OUTPUTS",        {  },              { "U32" }         )
register_cmd(0x14, 0xA1, "GET_STATUS",              {},                     { "U32" }         )
register_cmd(0x14, 0xA2, "IS_OVERLOADED",        {  },              { "U32" }         )
register_cmd(0x14, 0xA3, "NUM_LOG_SAMPLES",        {  },              { "U32" }         )
register_cmd(0x14, 0xA4, "NUM_LOG_WRAPS",        {  },              { "U32" }         )
register_cmd(0x14, 0xA5, "MAX_LOG_SAMPLES",        {  },              { "U32" }         )
register_cmd(0x14, 0xA6, "GET_SIM_MODE",        {  },              { "U32" }         )
register_cmd(0x14, 0xA7, "SET_DEFAULT_STOP_TIME",        {  },              {  }         )
register_cmd(0x14, 0xA8, "GET_XML_SIZE",        {  },              { "U32" }         )
register_cmd(0x14, 0xAC, "UNLOAD_APP",              {},                     {}                )

-- All 0x1E cmds:
register_cmd(0x1E, 0x0A, "SC_GET_DATA",        { "I32", "I32", "I32", "I32", "I32" },              { "ARR_F64" }         )
register_cmd(0x1E, 0x0F, "ADD_SCOPE",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x10, "REM_SCOPE",        { "U32" },              {  }         )
register_cmd(0x1E, 0x14, "SC_GET_START_TIME",        { "U32" },              { "F64" }         )
register_cmd(0x1E, 0x19, "SC_GET_TRIGGER_LEVEL",        { "U32" },              { "F64" }         )
register_cmd(0x1E, 0x1E, "SC_SET_TRIGGER_LEVEL",        { "U32", "F64" },              {  }         )
register_cmd(0x1E, 0x23, "SC_GET_TRIGGER_MODE",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x28, "SC_SET_TRIGGER_MODE",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x2D, "SC_GET_TRIGGER_SIGNAL",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x32, "SC_SET_TRIGGER_SIGNAL",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x37, "SC_GET_TRIGGER_SLOPE",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x3C, "SC_SET_TRIGGER_SLOPE",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x41, "SC_GET_TRIGGER_SCOPE",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x46, "SC_SET_TRIGGER_SCOPE",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x4B, "SC_GET_TRIGGER_SCOPE_SAMPLE",        { "U32" },              { "I32" }         )
register_cmd(0x1E, 0x50, "SC_SET_TRIGGER_SCOPE_SAMPLE",        { "U32", "I32" },              {  }         )
register_cmd(0x1E, 0x55, "SC_GET_DECIMATION",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x5A, "SC_SET_DECIMATION",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x64, "SC_GET_NUM_SAMPLES",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x65, "SC_SET_NUM_SAMPLES",        { "U32", "I32" },              {  }         )
register_cmd(0x1E, 0x66, "SC_GET_NUM_PRE_POST_SAMPLES",        { "U32" },              { "I32" }         )
register_cmd(0x1E, 0x68, "SC_SET_NUM_PRE_POST_SAMPLES",        { "U32", "I32" },              {  }         )
register_cmd(0x1E, 0x6C, "SC_SET_AUTO_RESTART",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x6A, "SC_GET_AUTO_RESTART",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x6E, "SC_GET_TYPE",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x78, "SC_ADD_SIG",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x79, "SC_REM_SIGNAL",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x7D, "SC_GET_STATE",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x7E, "SC_GET_GRID",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x7F, "SC_SET_GRID",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x80, "SC_GET_MODE",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x81, "SC_SET_MODE",        { "U32", "U32" },              {  }         )
register_cmd(0x1E, 0x82, "SC_GET_Y_LIMITS",        { "U32" },              { "F64", "F64" }         )
register_cmd(0x1E, 0x83, "SC_SET_Y_LIMITS",        { "U32", "F64", "F64" },              {  }         )
register_cmd(0x1E, 0x84, "SC_GET_VIEW_MODE",        {  },              { "U32" }         )
register_cmd(0x1E, 0x85, "SC_SET_VIEW_MODE",        { "U32" },              {  }         )
register_cmd(0x1E, 0x87, "SC_GET_SIGNAL_FORMAT",        { "U32", "U32" },              { "STR" }         )
register_cmd(0x1E, 0x8C, "SC_SET_SIGNAL_FORMAT",        { "U32", "U32", "STR" },              {  }         )
register_cmd(0x1E, 0x91, "SC_GET_NUM_SIGNALS",        { "U32" },              { "U32" }         )
register_cmd(0x1E, 0x96, "SC_START",        { "U32" },              {  }         )
register_cmd(0x1E, 0x97, "SC_STOP",        { "U32" },              {  }         )
register_cmd(0x1E, 0x98, "SC_SOFTWARE_TRIGGER",        { "U32" },              {  }         )
register_cmd(0x1E, 0x99, "GET_NUM_SCOPES",        {  },              { "U32" }         )
register_cmd(0x1E, 0x9A, "GET_NUM_HIDDEN_SCOPES",        {  },              { "U32" }         )
register_cmd(0x1E, 0x9B, "GET_SCOPES",        {  },              { "ARR_I32" }         )
register_cmd(0x1E, 0x9C, "GET_SCOPE_LIST",        {  },              { "ARR_I32" }         )
register_cmd(0x1E, 0x9D, "GET_HIDDEN_LIST",        {  },              { "ARR_I32" }         )
register_cmd(0x1E, 0x9E, "GET_HIDDEN_SCOPES",        {  },              { "ARR_I32" }         )
register_cmd(0x1E, 0x9F, "SC_GET_SIGNAL_LIST",        { "U32" },              { "ARR_I32" }         )
register_cmd(0x1E, 0xA0, "SC_GET_SIGNALS",        { "U32" },              { "ARR_I32" }         )
register_cmd(0x1E, 0xA1, "GET_SCOPE",        { "U32" },              { "F64", table.unpack(repeat_fmt("U32", 21))}         )
register_cmd(0x1E, 0xA2, "SET_SCOPE",        { "U32", "F64", table.unpack(repeat_fmt("U32", 19)) },              {  }         )


-- TODO: Add 0x46, 0x5a, 0xbe, 0x64 cmds

----------------------------------------------------------------------
-- Protocol / field declarations
----------------------------------------------------------------------

xpc = Proto("xpc", "MathWorks xPC Target API (xpcapi)")

local f_magic    = ProtoField.uint16("xpc.magic",   "Magic",    base.HEX)
local f_cmd      = ProtoField.uint8 ("xpc.cmd",     "Cmd",      base.HEX)
local f_subcmd   = ProtoField.uint8 ("xpc.subcmd",  "Subcmd",   base.HEX)
local f_name     = ProtoField.string("xpc.name",    "Name")
local f_length   = ProtoField.uint32("xpc.length",  "Payload Length", base.DEC)
local f_dir      = ProtoField.string("xpc.direction","Direction")

local f_u8       = ProtoField.uint8  ("xpc.u8",  "uint8")
local f_i8       = ProtoField.int8   ("xpc.i8",  "int8")
local f_u16      = ProtoField.uint16 ("xpc.u16", "uint16")
local f_i16      = ProtoField.int16  ("xpc.i16", "int16")
local f_u32      = ProtoField.uint32 ("xpc.u32", "uint32")
local f_i32      = ProtoField.int32  ("xpc.i32", "int32")
local f_u64      = ProtoField.uint64 ("xpc.u64", "uint64")
local f_i64      = ProtoField.int64  ("xpc.i64", "int64")
local f_float    = ProtoField.float  ("xpc.f32", "float")
local f_double   = ProtoField.double ("xpc.f64", "double")
local f_string   = ProtoField.string ("xpc.str", "string")
local f_arrlen   = ProtoField.uint32 ("xpc.arrlen", "array length")
local f_strlen   = ProtoField.uint32 ("xpc.strlen", "string length")
local f_unknown  = ProtoField.bytes  ("xpc.unknown", "unparsed data")

xpc.fields = {
    f_magic, f_cmd, f_subcmd, f_name, f_length, f_dir,
    f_u8, f_i8, f_u16, f_i16, f_u32, f_i32, f_u64, f_i64,
    f_float, f_double, f_string, f_arrlen, f_strlen, f_unknown,
}

local HEADER_LEN = 8

----------------------------------------------------------------------
-- Helpers
----------------------------------------------------------------------

local function rd_uint(buf)
    return (ENDIANNESS == "le") and buf:le_uint() or buf:uint()
end
local function rd_int(buf)
    return (ENDIANNESS == "le") and buf:le_int() or buf:int()
end
local function rd_float(buf)
    return (ENDIANNESS == "le") and buf:le_float() or buf:float()
end
local function rd_uint64(buf)
    return (ENDIANNESS == "le") and buf:le_uint64() or buf:uint64()
end
local function rd_int64(buf)
    return (ENDIANNESS == "le") and buf:le_int64() or buf:int64()
end

local function buffer(offset, len, context)
    context = context or "unknown"
    
    if offset < 0 then
        error(string.format("Negative offset %d in %s", offset, context))
    end
    
    if len < 0 then
        error(string.format("Negative length %d in %s", len, context))
    end
    
    local buf_len = buffer:len()
    
    if offset + len > buf_len then
        error(string.format(
            "Buffer overrun in %s: offset=%d, len=%d, buf_len=%d (would read up to byte %d)",
            context, offset, len, buf_len, offset + len
        ))
    end
    
    return buffer(offset, len)
end

-- Adds a multi-byte ProtoField to the tree with display byte-order
-- matching ENDIANNESS. Wireshark's plain tree:add() renders multi-byte
-- fields big-endian by default regardless of how you read them
-- elsewhere in Lua -- add_le() is needed to display little-endian
-- values correctly. Single-byte fields (cmd/subcmd) don't need this.
local function add_field(tree_obj, field, range)
    if ENDIANNESS == "le" then
        return tree_obj:add_le(field, range)
    else
        return tree_obj:add(field, range)
    end
end

local SCALAR_SIZES = {
    uint8=1, int8=1, uint16=2, int16=2, uint32=4, int32=4,
    uint64=8, int64=8, float=4, double=8,
}

local function parse_scalar(buffer, offset, basetype, label, subtree)
    if basetype == "uint8" then
        subtree:add(f_u8, buffer(offset,1)):set_text(
            string.format("%s: %d (uint8)", label, buffer(offset,1):uint()))
    elseif basetype == "int8" then
        subtree:add(f_i8, buffer(offset,1)):set_text(
            string.format("%s: %d (int8)", label, buffer(offset,1):int()))
    elseif basetype == "uint16" then
        subtree:add(f_u16, buffer(offset,2)):set_text(
            string.format("%s: %d (uint16)", label, rd_uint(buffer(offset,2))))
    elseif basetype == "int16" then
        subtree:add(f_i16, buffer(offset,2)):set_text(
            string.format("%s: %d (int16)", label, rd_int(buffer(offset,2))))
    elseif basetype == "uint32" then
        subtree:add(f_u32, buffer(offset,4)):set_text(
            string.format("%s: %d (uint32)", label, rd_uint(buffer(offset,4))))
    elseif basetype == "int32" then
        subtree:add(f_i32, buffer(offset,4)):set_text(
            string.format("%s: %d (int32)", label, rd_int(buffer(offset,4))))
    elseif basetype == "uint64" then
        subtree:add(f_u64, buffer(offset,8)):set_text(
            string.format("%s: %s (uint64)", label, tostring(rd_uint64(buffer(offset,8)))))
    elseif basetype == "int64" then
        subtree:add(f_i64, buffer(offset,8)):set_text(
            string.format("%s: %s (int64)", label, tostring(rd_int64(buffer(offset,8)))))
    elseif basetype == "float" then
        subtree:add(f_float, buffer(offset,4)):set_text(
            string.format("%s: %f (float)", label, rd_float(buffer(offset,4))))
    elseif basetype == "double" then
        local le = (ENDIANNESS == "le")
        local v = le and buffer(offset,8):le_float() or buffer(offset,8):float()
        subtree:add(f_double, buffer(offset,8)):set_text(
            string.format("%s: %f (double)", label, v))
    else
        error("parse_scalar: unknown basetype '" .. tostring(basetype) .. "'")
    end
    return offset + SCALAR_SIZES[basetype]
end

-- Returns new offset, or nil if not enough data is available (avail bound).
local function parse_field(buffer, offset, spec, subtree, avail)
    if spec.type == "string" then
        if avail - offset < 4 then return nil end
        local len = rd_uint(buffer(offset,4))
        if avail - (offset + 4) < len then return nil end
        -- Length prefix and string content get their own separate tree
        -- items (mirroring how the array branch below already does it),
        -- so Wireshark's own string-field validation never sees the raw
        -- binary length bytes as part of the "string" -- that mismatch
        -- was what triggered the earlier "trailing stray characters"
        -- expert warning; the length field itself was never wrong.
        local str_subtree = subtree:add(f_strlen, buffer(offset,4))
        str_subtree:set_text(string.format("%s: string (len=%d)", spec.name, len))
        if len > 0 then
            local txt = buffer(offset+4, len):string()
            str_subtree:add(f_string, buffer(offset+4, len)):set_text(
                string.format("text: \"%s\"", txt))
        end
        return offset + 4 + len

    elseif spec.type == "array" then
        if avail - offset < 4 then return nil end
        local count = rd_uint(buffer(offset,4))
        local esz = SCALAR_SIZES[spec.elem]
        if esz == nil then
            error("parse_field: unsupported array element type '" .. tostring(spec.elem) .. "'")
        end
        if avail - (offset + 4) < count * esz then return nil end
        local arr_subtree = subtree:add(f_arrlen, buffer(offset,4))
        arr_subtree:set_text(string.format("%s: array of %d x %s", spec.name, count, spec.elem))
        local pos = offset + 4
        for i = 1, count do
            pos = parse_scalar(buffer, pos, spec.elem,
                string.format("%s[%d]", spec.name, i - 1), arr_subtree)
        end
        return pos

    else
        local need = SCALAR_SIZES[spec.type]
        if need == nil then
            error("parse_field: unknown field type '" .. tostring(spec.type) .. "'")
        end
        if avail - offset < need then return nil end
        return parse_scalar(buffer, offset, spec.type, spec.name, subtree)
    end
end

----------------------------------------------------------------------
-- Direction detection
----------------------------------------------------------------------

-- Returns true (request), false (response), or nil (unknown -- port
-- didn't match either side, e.g. XPC_SERVER_PORT is misconfigured).
local function is_request(pinfo)
    if pinfo.dst_port == XPC_SERVER_PORT then return true end
    if pinfo.src_port == XPC_SERVER_PORT then return false end
    return nil
end

----------------------------------------------------------------------
-- Dissect one complete message (header + payload) directly on the
-- original Tvb, starting at msg_start. No child Tvb is created.
----------------------------------------------------------------------

local function dissect_message(buffer, msg_start, pinfo, tree, req)
    local payload_len = rd_uint(buffer(msg_start + 4, 4))
    local cmd = buffer(msg_start + 2, 1):uint()
    local subcmd = buffer(msg_start + 3, 1):uint()
    local msg_total = HEADER_LEN + payload_len

    local root = tree:add(xpc, buffer(msg_start, msg_total), "xPC Message")
    add_field(root, f_magic, buffer(msg_start, 2))
    root:add(f_cmd, buffer(msg_start + 2, 1))
    root:add(f_subcmd, buffer(msg_start + 3, 1))
    add_field(root, f_length, buffer(msg_start + 4, 4))

    local entry = CMD_TABLE[cmd] and CMD_TABLE[cmd][subcmd]
    local cmd_name = entry and entry.name or string.format("Unknown (0x%02X/0x%02X)", cmd, subcmd)
    root:add(f_name, buffer(msg_start + 2, 2), cmd_name):set_generated(true)

    local dir_str
    if req == true then dir_str = "Request"
    elseif req == false then dir_str = "Response"
    else dir_str = "Unknown (set XPC_SERVER_PORT)" end
    root:add(f_dir, buffer(msg_start, 0), dir_str):set_generated(true)

    pinfo.cols.protocol = "XPC"
    pinfo.cols.info = string.format("%s [%s]  cmd=0x%02X/0x%02X  len=%d",
        cmd_name, dir_str, cmd, subcmd, payload_len)

    if payload_len == 0 then
        return
    end

    local payload_start = msg_start + HEADER_LEN
    local avail = payload_start + payload_len

    if req == nil then
        -- Direction unknown -- can't safely pick req vs resp layout.
        root:add(f_unknown, buffer(payload_start, payload_len),
            "Payload (direction unknown, raw bytes -- set XPC_SERVER_PORT)")
        return
    end

    if entry == nil then
        root:add(f_unknown, buffer(payload_start, payload_len),
            "Payload (cmd/subcmd not in CMD_TABLE, raw bytes)")
        return
    end

    local spec_list = req and entry.req or entry.resp
    local payload_tree = root:add(xpc, buffer(payload_start, payload_len), "Payload")
    local offset = payload_start

    for _, spec in ipairs(spec_list) do
        local new_offset = parse_field(buffer, offset, spec, payload_tree, avail)
        if new_offset == nil then
            payload_tree:add(f_unknown, buffer(offset, avail - offset),
                "Truncated / malformed field: " .. spec.name)
            offset = avail
            break
        end
        offset = new_offset
    end

    if offset < avail then
        payload_tree:add(f_unknown, buffer(offset, avail - offset),
            "Trailing unparsed bytes")
    end
end

----------------------------------------------------------------------
-- TCP-stream-aware entry point: handles reassembly and multiple
-- messages per TCP segment
----------------------------------------------------------------------

local function dissect(buffer, pinfo, tree)  
    local total_len = buffer:len()
    local offset = 0
    local req = is_request(pinfo)

    while offset < total_len do
        local remaining = total_len - offset

        if remaining < HEADER_LEN then
            pinfo.desegment_offset = offset
            pinfo.desegment_len = HEADER_LEN - remaining
            return
        end

        local header = buffer(offset, HEADER_LEN)
        local magic = rd_uint(header(0,2))
        if magic ~= MAGIC then
            return -- not our protocol / desynced stream
        end

        local payload_len = rd_uint(header(4,4))
        if payload_len > MAX_PAYLOAD_LEN then
            -- Almost certainly a false-positive magic match or a
            -- misparsed length field, not a real oversized message.
            -- Bail out instead of requesting a huge reassembly.
            return
        end
        local msg_total = HEADER_LEN + payload_len

        if remaining < msg_total then
            pinfo.desegment_offset = offset
            pinfo.desegment_len = msg_total - remaining
            return
        end

        dissect_message(buffer, offset, pinfo, tree, req)
        offset = offset + msg_total
    end
end

function xpc.dissector(buffer, pinfo, tree)
    local success, result = pcall(dissect, buffer, pinfo, tree)

    if not success then
        pinfo.cols.protocol = "XPC"
        pinfo.cols.info = "ERROR: " .. tostring(result)
    return
  end
end


----------------------------------------------------------------------
-- Registration
----------------------------------------------------------------------

-- Heuristic: try this dissector against any TCP payload; claim it only if
-- the magic number matches.
local function heuristic_checker(buffer, pinfo, tree)
    if buffer:len() < HEADER_LEN then return false end
    local magic = (ENDIANNESS == "le") and buffer(0,2):le_uint() or buffer(0,2):uint()
    if magic ~= MAGIC then return false end
    -- Extra sanity check before claiming: reject implausible length
    -- values up front so a coincidental magic match on unrelated
    -- traffic doesn't get treated as ours.
    local payload_len = rd_uint(buffer(4,4))
    if payload_len > MAX_PAYLOAD_LEN then return false end
    xpc.dissector(buffer, pinfo, tree)
    return true
end

xpc:register_heuristic("tcp", heuristic_checker)

-- Also register on the known xPC Target server port for a fast, certain
-- path (and so it still works if you ever disable the heuristic).
local tcp_port_table = DissectorTable.get("tcp.port")
tcp_port_table:add(XPC_SERVER_PORT, xpc)
