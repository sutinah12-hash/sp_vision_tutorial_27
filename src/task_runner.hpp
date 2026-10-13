#pragma once

// Shared executable entry; no hardware is opened unless --live is explicit.
int run_task(int task, int argc, char * argv[]);
