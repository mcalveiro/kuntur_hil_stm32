#include "autopilot_state.h"
#include <string.h>

void autopilot_state_reset(Autopilot_State_t *s) {
    memset(s, 0, sizeof(*s));
}
