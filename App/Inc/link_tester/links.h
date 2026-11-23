#ifndef LINKS_H
#define LINKS_H

#include "link_manager.h"

#define LINKS_MAX 8u

/* Create every driver and manager the web page can start. Call once, after the
   HAL and CubeMX peripheral init, and before cmd_init(). */
void            links_init(void);
unsigned        links_count(void);
const char*     links_id(unsigned i);
link_manager_t* links_mgr(unsigned i);
int             links_index(const char *id);   /* -1 if the id is unknown */
void            links_step_all(void);          /* call every main-loop iteration */

#endif /* LINKS_H */