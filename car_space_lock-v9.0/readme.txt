1.把POWER_SAVING宏打开，让系统有机会进入低功耗
2.删除周期性处理函数2， 只保留周期性处理函数1
3.把周期性处理 函数耗时的动作移出到SimpleBLEPeripheral_ProcessEvent（）。