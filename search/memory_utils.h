/**
 * This file provide a function to check memory usage of the calling
 * process. The codes should be working on Both Mac and Linux, Windows
 * is not supported yet.
 */


/* takes two doubles by reference, attempts to read the system-dependent
 * data for a process' virtual memory size and resident set size
 * (not realy working for us), and return the results in KB.
 * On failure, returns 0.0, 0.0
 */
void process_mem_usage(double &vm_usage, double &resident_set);
