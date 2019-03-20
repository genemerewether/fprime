SRC = src/address.cpp                src/io_thread.cpp       src/pgm_sender.cpp    src/router.cpp           src/tipc_connecter.cpp \
src/client.cpp                 src/ipc_address.cpp     src/pgm_socket.cpp    src/scatter.cpp          src/tipc_listener.cpp \
src/clock.cpp                  src/ipc_connecter.cpp   src/pipe.cpp          src/select.cpp           src/trie.cpp \
src/ctx.cpp                    src/ipc_listener.cpp    src/plain_client.cpp  src/server.cpp           src/udp_address.cpp \
           src/ip.cpp              src/plain_server.cpp  src/session_base.cpp     src/udp_engine.cpp \
           src/kqueue.cpp          src/poll.cpp          src/signaler.cpp         src/v1_decoder.cpp \
src/dealer.cpp                 src/lb.cpp              src/poller_base.cpp   src/socket_base.cpp      src/v1_encoder.cpp \
src/decoder_allocators.cpp     src/mailbox.cpp         src/pollset.cpp       src/socket_poller.cpp    src/v2_decoder.cpp \
src/devpoll.cpp                src/mailbox_safe.cpp    src/precompiled.cpp   src/socks_connecter.cpp  src/v2_encoder.cpp \
src/dgram.cpp                  src/mechanism.cpp       src/proxy.cpp         src/socks.cpp            src/vmci_address.cpp \
src/dish.cpp                   src/metadata.cpp        src/pub.cpp           src/stream.cpp           src/vmci_connecter.cpp \
src/dist.cpp                   src/msg.cpp             src/pull.cpp          src/stream_engine.cpp    src/vmci.cpp \
src/epoll.cpp                  src/mtrie.cpp           src/push.cpp          src/sub.cpp              src/vmci_listener.cpp \
src/err.cpp                    src/norm_engine.cpp     src/radio.cpp         src/tcp_address.cpp      src/xpub.cpp \
src/fq.cpp                     src/null_mechanism.cpp  src/random.cpp        src/tcp_connecter.cpp    src/xsub.cpp \
src/gather.cpp                 src/object.cpp          src/raw_decoder.cpp   src/tcp.cpp              src/zmq.cpp \
src/gssapi_client.cpp          src/options.cpp         src/raw_encoder.cpp   src/tcp_listener.cpp      \
src/gssapi_mechanism_base.cpp  src/own.cpp             src/reaper.cpp        src/thread.cpp \
src/gssapi_server.cpp          src/pair.cpp            src/rep.cpp           src/timers.cpp \
src/io_object.cpp              src/pgm_receiver.cpp    src/req.cpp           src/tipc_address.cpp \
tools/curve_keygen.cpp src/curve_client.cpp src/curve_server.cpp src/zmq_utils.cpp src/tweetnacl.c

# src/curve_client.cpp src/curve_server.cpp src/zmq_utils.cpp