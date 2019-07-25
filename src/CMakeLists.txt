add_library(enc28j60 STATIC init.c control.c registers.c tx.c rx.c)
target_include_directories(enc28j60 PUBLIC ../include)
