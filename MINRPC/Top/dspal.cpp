#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>

#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#define MAX 10

int buffer[MAX];
int fill  = 0; 
int use   = 0;
volatile int loops = 0;

void put(int value) {
    buffer[fill] = value;    // line F1
    fill = (fill + 1) % MAX; // line F2
}

int get() {
    int tmp = buffer[use];   // line G1
    use = (use + 1) % MAX;   // line G2
    return tmp;
}

sem_t empty;
sem_t full;
sem_t mutex;

int rpcBuffer[MAX];
int rpcFill  = 0; 
int rpcUse   = 0;

void rpcPut(int value) {
    rpcBuffer[rpcFill] = value;    // line F1
    rpcFill = (rpcFill + 1) % MAX; // line F2
}

int rpcGet() {
    int tmp = rpcBuffer[rpcUse];   // line G1
    rpcUse = (rpcUse + 1) % MAX;   // line G2
    return tmp;
}

sem_t rpcEmpty;
sem_t rpcFull;
sem_t rpcMutex;

void *producer(void *arg) {
    int i;
    for (i = 0; i < loops; i++) {
        sem_wait(&empty);           // line P1
        sem_wait(&mutex);           // line P1.5 (MOVED THE MUTEX TO HERE ...)
        put(i);                     // line P2
        sem_post(&mutex);           // line P2.5 (... AND TO HERE)
        sem_post(&full);            // line P3
        
        sem_wait(&rpcEmpty);           // line P1
        sem_wait(&rpcMutex);           // line P1.5 (MOVED THE MUTEX TO HERE ...)
        rpcPut(i);                     // line P2
        sem_post(&rpcMutex);           // line P2.5 (... AND TO HERE)
        sem_post(&rpcFull);            // line P3
    }
    usleep(1000);
}

void *consumer(void *arg) {
    int i;
    for (i = 0; i < loops; i++) {
        sem_wait(&full);            // line C1
        sem_wait(&mutex);           // line C1.5 (MOVED THE MUTEX TO HERE ...)
        int tmp = get();            // line C2
        sem_post(&mutex);           // line C2.5 (... AND TO HERE)
        sem_post(&empty);           // line C3
        FARF(ALWAYS, "%d\n", tmp);
    }
}

int hexref_init() {
    FARF(ALWAYS, "hexref_init");
    return 0;
}

int hexref_arm() {
    FARF(ALWAYS, "hexref_arm");
    return 0;
}

int hexref_run() {
    FARF(ALWAYS, "hexref_run");

    return 0;
}

int hexref_cycle(unsigned int cycles) {
    FARF(ALWAYS, "hexref_cycle");
    loops = cycles;
    
    sem_init(&empty, 0, MAX); // MAX buffers are empty to begin with...
    sem_init(&full, 0, 0);    // ... and 0 are full
    sem_init(&mutex, 0, 1);   // mutex = 1 because it is a lock (NEW LINE)

    pthread_t pid, cid;
    pthread_create(&pid, NULL, producer, NULL); 
    pthread_create(&cid, NULL, consumer, NULL); 
    pthread_join(pid, NULL); 
    pthread_join(cid, NULL);
    
    return 0;
}

int hexref_wait() {
    FARF(ALWAYS, "hexref_wait");
    return 0;
}

int hexref_fini() {
    FARF(ALWAYS, "hexref_fini");
    return 0;
}

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    bytes = 0;
    return 0;
}

int hexref_rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
    sem_wait(&rpcFull);            // line C1
    sem_wait(&rpcMutex);           // line C1.5 (MOVED THE MUTEX TO HERE ...)
    int tmp = rpcGet();            // line C2
    sem_post(&rpcMutex);           // line C2.5 (... AND TO HERE)
    sem_post(&rpcEmpty);           // line C3
    FARF(ALWAYS, "rpc %d\n", tmp);

    bytes = 0;
    return 0;
}

int hexref_rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return 0;
}

int hexref_rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    return 0;
}
