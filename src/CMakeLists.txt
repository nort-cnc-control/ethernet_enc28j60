add_library(enc28j60 STATIC init.c control.c registers.c tx.c rx.c eth_codec.c)
target_include_directories(enc28j60 PUBLIC ../include)
