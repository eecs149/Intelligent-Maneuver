#!/usr/bin/env python
import select
import socket
import sys
from collections import defaultdict, deque


def main(argv):
    if len(argv) != 2 and len(argv) != 3:
        print "Usage: python memdb.py <port> [-v]"
        return

    verbose = len(argv) == 3 and argv[2] == '-v'

    host = ''
    port = int(argv[1])
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(5)

    inputs = set([server])
    data = defaultdict(deque)
    waiters = defaultdict(deque)
    
    while True:
        try:
            inputready, outputready, exceptready = select.select(inputs, [], [])
        except KeyboardInterrupt:
            if verbose: print 'quitting'
            break

        for s in inputready:
            if s == server:
                client, address = server.accept()
                inputs.add(client)
                if verbose: print 'client connected:', client
            else:
                datum = s.recv(1024)
                if not datum:
                    s.close()
                    inputs.remove(s)
                    if verbose: print 'client disconnected:', client
                    continue

                for datum in datum.splitlines():
                    if verbose: print datum

                    tokens = datum.strip().split(' ')
                    if len(tokens) != 2 and len(tokens) !=3: continue
                    mode = tokens[0]
                    key = tokens[1]
                    
                    if mode == 'put':
                        if len(tokens) != 3: continue
                        val = tokens[2]
                        if len(waiters[key]) > 0:
                            waiters[key].popleft().send(val)
                        else:
                            data[key].append(val)
                    elif mode == 'get':
                        if len(tokens) != 2: continue
                        if len(data[key]) == 0:
                            waiters[key].append(s)
                        else:
                            s.send(data[key].popleft())
                    elif mode == 'count':
                        if len(tokens) != 2: continue
                        s.send(str(len(data[key])))
                    elif mode == 'clear':
                        if len(tokens) != 2: continue
                        data[key].clear()
                    elif mode == 'tryget':
                        if len(tokens) != 2: continue
                        if len(data[key]) == 0:
                            s.send('-none-')
                        else:
                            s.send(data[key].popleft())


if __name__ == '__main__':
    main(sys.argv)
