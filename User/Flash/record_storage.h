#ifndef __RECORD_STORAGE_H
#define __RECORD_STORAGE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif


//单条输入记录允许保存的最大字节数
#define RECORD_STORAGE_TEXT_MAX_BYTES  128
//初始化flash扫描模块并扫描已有记录
uint8_t RecordStorage_Init(void);
//增加一条记录
uint8_t RecordStorage_Append(const uint8_t *text,uint16_t text_length);
/* 下一次追加是否需要先擦除128 KB扇区。 */
uint8_t RecordStorage_NeedsErase(void);
//获取当前有效历史记录数量
uint8_t RecordStorage_GetRecentCount(void);
//读取最近的记录
uint8_t RecordStorage_ReadRecent(uint8_t recent_index,char *destination,size_t destination_size);


#ifdef __cplusplus
}
#endif
#endif

