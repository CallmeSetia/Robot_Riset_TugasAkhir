void RTOS_Init() {
  
  xTaskCreatePinnedToCore(
    KomunkasiMasterSlaveTask,   /* Task function. */
    "Komunkasi Master Slave Task",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */


}
