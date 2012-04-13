import select
import socket
import sys
from collections import defaultdict, deque

def main(argv):
    if len(argv) != 2:
        print "Usage: python memdb.py <port>"
        return

    host = ''
    port = int(argv[1])
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(5)

    inputs = set([server])
    data = defaultdict(deque)
    waiters = defaultdict(deque)
    
    while True:
        inputready, outputready, exceptready = select.select(inputs, [], [])
        for s in inputready:
            if s == server:
                client, address = server.accept()
                inputs.add(client)
            else:
                datum = s.recv(1024)
                if not datum:
                    s.close()
                    inputs.remove(s)
                    continue

                for datum in datum.splitlines():
                    tokens = datum.split(' ')
                    if len(tokens) != 2 and len(tokens) !=3: continue
                    mode = tokens[0]
                    
                    if mode == 'put':
                        if len(tokens) != 3: continue
                        key = tokens[1].strip()
                        val = tokens[2].strip()
                        print "put", key, val
                        if len(waiters[key]) > 0:
                            w = waiters[key].popleft()
                            w.send(val)
                        else:
                            data[key].append(val)
                    elif mode == 'get':
                        if len(tokens) != 2: continue
                        key = tokens[1].strip()
                        print "get", key
                        if len(data[key]) == 0:
                            waiters[key].append(s)
                        else:
                            s.send(data[key].popleft())
                    elif mode == 'count':
                        if len(tokens) != 2: continue
                        key = tokens[1].strip()
                        s.send(str(len(data[key])))


if __name__ == '__main__':
    main(sys.argv)
