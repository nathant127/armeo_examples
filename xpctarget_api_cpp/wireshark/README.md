```
wireshark -X lua_script:xpc.lua main_menu.pcapng
```
```
tshark -X lua_script:xpc.lua -r main_menu.pcapng -Y xpc -V
```