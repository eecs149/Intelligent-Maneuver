#include <errno.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>

static int db_connect(const char* port) {
    int sock;
    struct addrinfo hints, *servinfo, *p;

    vp_os_memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo("localhost", port, &hints, &servinfo) != 0) {
        fputs("getaddrinfo failed\n", stderr);
        return -1;
    }

    for (p = servinfo; p != NULL; p = p->ai_next) {
        sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (connect(sock, p->ai_addr, p->ai_addrlen) == -1) {
            close(sock);
            continue;
        }

        break;
    }

    if (!p)
    {
        fputs("client failed to connect.\n", stderr);
        return -1;
    }

    return sock;
}

static void db_close(int db) {
    close(db);
}

static void _db_send(int db, const char *format, ...) {
    char buf[1024];
    int size;

    va_list args;
    va_start(args, format);
    size = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    send(db, buf, size, 0);
}
    

static void db_printf(int db, const char *key, const char *format, ...) {
    char buf[1024];
    int size;

    size = snprintf(buf, sizeof(buf), "put %s ", key);

    va_list args;
    va_start(args, format);
    size += vsnprintf(buf+size, sizeof(buf)-size-1, format, args);
    va_end(args);

    buf[size] = '\n';
    buf[size+1] = '\0';
    send(db, buf, size+1, 0);
}

static int db_get(int db, const char *key, char *out, int size) {
    _db_send(db, "get %s\n", key);
    int ret_size = recv(db, out, size, 0);
    out[ret_size] = '\0';
    return ret_size;
}

static int db_tryget(int db, const char *key, char *out, int size) {
    _db_send(db, "tryget %s\n", key);
    int ret_size = recv(db, out, size, 0);
    out[ret_size] = '\0';
    if (!strcmp(out, "-none-"))
        return -1;
    return ret_size;
}

static int db_scanf(int db, const char *key, const char *format, ...) {
    char buf[1024];
    int size = db_get(db, key, buf, sizeof(buf));

    va_list args;
    va_start(args, format);
    int num_read = vsscanf(buf, format, args);
    va_end(args);

    return num_read;
}

static int db_tryscanf(int db, const char *key, const char *format, ...) {
    char buf[1024];
    int size = db_tryget(db, key, buf, sizeof(buf));
    if (size < 0)
        return -1;

    va_list args;
    va_start(args, format);
    int num_read = vsscanf(buf, format, args);
    va_end(args);

    return num_read;
}

static int db_count(int db, const char *key) {
    char buf[16] = "";
    _db_send(db, "count %s\n", key);
    recv(db, buf, sizeof(buf), 0);
    return atoi(buf);
}

static void db_clear(int db, const char *key) {
    _db_send(db, "clear %s\n", key);
}

