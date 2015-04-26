#include <stdio.h>


// a b c d
//   0 1 2 3
//   0
#define SIZE 4

int tx_head, tx_tail;

int size() {
    return (tx_head >= tx_tail) ? tx_head - tx_tail : tx_head + SIZE - tx_tail;
}

void put() {
    if (size() == SIZE - 1) {
        printf("can't put!\n");
        return;
    }
    if (++tx_head == SIZE) tx_head = 0;
}

void emit() {
    if (size() == 0) {
        printf("can't emit!\n");
        return;
    }
    if (++tx_tail == SIZE) tx_tail = 0;
}

void du() {
    printf("head=%d tail=%d size=%d\n", tx_head, tx_tail, size());
}

main()
{
    du();
    put();
    du();
    put();
    du();
    put();
    du();
    put();
    du();
    emit(); du();
    emit(); du();
    emit(); du();
    emit(); du();

    put(); du();
    put(); du();
    put(); du();
    put(); du();
}
