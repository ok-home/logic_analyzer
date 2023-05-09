#pragma once

#ifdef __cplusplus
extern "C" {
#endif
// create & start ws server - if server already running - skip & go to la_ws - register uri handlers
void logic_analyzer_ws_server(void);

#ifdef __cplusplus
}
#endif