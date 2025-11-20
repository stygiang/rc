#pragma once

// RC -> motor control bridge on the Master.
// Reads RC receiver channels and sends motor + direction commands to Partner.

void rcControlSetup();
void rcControlLoop();
void rcControlEnable();
void rcControlDisable();
bool rcControlIsEnabled();
