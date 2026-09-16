#include <stdio.h>
#include "types.h"
#include "proc_owner.h"

static u32 failures;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #c); failures++; \
} } while (0)

int main(void)
{
    struct proc_owner_record record = {0};
    CHECK(sizeof(record) == 64 && _Alignof(struct proc_owner_record) == 64);
    proc_owner_publish(&record, 1, 2);
    CHECK(proc_owner_load(&record) == 2);
    CHECK(proc_owner_claim(&record, 1, 2, 1));
    CHECK(!proc_owner_claim(&record, 1, 2, 3));
    CHECK(!proc_owner_claim(&record, 1, proc_owner_load(&record), 3));
    CHECK(!proc_owner_finish(&record, 1, 3, 3));
    CHECK(proc_owner_finish(&record, 1, 1, 1));
    CHECK(proc_owner_load(&record) == 1);
    proc_owner_publish(&record, 2, 1);
    CHECK(!proc_owner_claim(&record, 1, 1, 2));
    CHECK(proc_owner_claim(&record, 2, 1, 2));
    proc_owner_publish(&record, 3, 3);
    CHECK(!proc_owner_finish(&record, 2, 2, 1));
    CHECK(proc_owner_load(&record) == 3);
    CHECK(proc_owner_claim(&record, 3, 3, 1));
    CHECK(proc_owner_finish(&record, 3, 1, 3));
    printf("Process owner: %u failures\n", failures);
    return failures ? 1 : 0;
}
