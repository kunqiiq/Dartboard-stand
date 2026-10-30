#include "record_storage.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "iwdg.h"
#include "main.h"

//Flash扇区配置
#define RECORD_STORAGE_SECTOR_A_BASE    0x080C0000L
#define RECORD_STORAGE_SECTOR_B_BASE    0x080E0000L

#define RECORD_STORAGE_SECTOR_A_NUMBER  FLASH_SECTOR_10
#define RECORD_STORAGE_SECTOR_B_NUMBER  FLASH_SECTOR_11

#define RECORD_STORAGE_SECTOR_SIZE      0x00020000L
#define RECORD_STORAGE_VOLTAGE_RANGE    FLASH_VOLTAGE_RANGE_3


/* ======================== 数据格式 ======================== */
#define RECORD_STORAGE_MAGIC    0x52454344L
#define RECORD_STORAGE_COMMIT   0x434F4D4DL
#define RECORD_STORAGE_VERSION  1

#define RECORD_STORAGE_RECENT_MAX    4
//flash删除后的默认值
#define RECORD_STORAGE_ERASED_WORD   0xFFFFFFFFL


typedef  struct
{
    uint32_t magic;
    uint32_t sequence;
    
    uint16_t text_length;
    uint16_t verison;
    
    uint8_t text[RECORD_STORAGE_TEXT_MAX_BYTES];
    
    uint32_t crc32;
    
    uint32_t commit;
}RecordStorage_Entry_t;

/*
 * 每个128 KB扇区可以保存约885条记录。
 */
#define RECORD_STORAGE_SLOT_COUNT          \
    (RECORD_STORAGE_SECTOR_SIZE /          \
     sizeof(RecordStorage_Entry_t))
         
     
/* ======================== 扇区运行状态 ======================== */
     
typedef struct
{
     uint32_t base_address;
     uint32_t sector_number;
     
     uint32_t first_erased_address;
     uint32_t maxmum_sequence;
     uint32_t valid_count;
     uint8_t has_valid_record;
    
}RecordStorage_SectorState_t;

static RecordStorage_SectorState_t record_storage_sector[2];

//当前活动扇区 0 = sector10，1 = sector11
static uint8_t record_storage_active_sector;
//下一个准备写入的flash地址
static uint32_t record_storage_next_address;
//下一条记录使用的递增序号
static uint32_t record_storage_next_sequence;
//模块是否完成初始化
static uint8_t record_storage_initialized;

/* ======================== 最近四条RAM缓存 ======================== */
static char record_storage_recent_text[RECORD_STORAGE_RECENT_MAX][RECORD_STORAGE_TEXT_MAX_BYTES + 1];
static uint16_t record_storage_recent_length[RECORD_STORAGE_RECENT_MAX];
static uint32_t record_storage_recent_sequence[RECORD_STORAGE_RECENT_MAX];
static uint8_t record_storage_recent_count;


/* ======================== CRC32 ======================== */

/*
 * CRC-32/IEEE 802.3
 *
 * Polynomial = 0xEDB88320
 * Init       = 0xFFFFFFFF
 * XorOut     = 0xFFFFFFFF
 */
 static uint32_t RecordStorage_CalculateCrc32(const uint8_t *data,size_t length)
 {
    uint32_t crc;
     uint32_t bit_index;
     size_t byte_index;
     
     crc = 0xFFFFFFFFL;
     
     if(data == NULL)
     {
        return 0;
     }
     for(byte_index = 0;byte_index < length;++byte_index)
     {
        crc ^= (uint32_t)data[byte_index];
         for(bit_index = 0;bit_index < 8;++bit_index)
         {
            if((crc & 1L) != 0)
            {
                crc = (crc >> 1) ^ 0xEDB88320L;
            }
            else
            {
                crc >>= 1;
            }
         }
     }
     return crc ^ 0xFFFFFFFFL;
 }

/* ======================== 序号比较 ======================== */
 //使用有符号差值比较递增讯号，允许uint32序号会回绕
 static uint8_t RecordStorage_SequenceIsNewer(uint32_t first,uint32_t second)
 {
     return (((int32_t)(first - second)) > 0) ? 1 : 0;
 }
 
 static uint32_t RecordStorage_GetNextSequence(uint32_t current)
 {
    uint32_t next;
     next = current + 1;
     
     if((next == 0) || (next == 0xFFFFFFFFL))
     {
        next = 1L;
     }
    return next;
 }


/* ======================== Flash数据检查 ======================== */
 static uint8_t RecordStorage_IsSlotErased(uint32_t address)
 {
    const uint32_t *word_pointer;
     uint32_t word_count;
     uint32_t index;
     
     word_pointer = (const uint32_t *)address;
     word_count = (uint32_t)(sizeof(RecordStorage_Entry_t) / (sizeof(uint32_t)));
     
     for(index = 0;index < word_count;++index)
     {
        if(word_pointer[index] != RECORD_STORAGE_ERASED_WORD)
        {
            return 0;
        }
     }
     return 1;
 }
 
 static uint8_t RecordStorage_IsSectorErased(uint32_t base_address)
{
    const uint32_t *word_pointer;
    uint32_t word_count;
    uint32_t index;

    word_pointer = (const uint32_t *)base_address;

    word_count = RECORD_STORAGE_SECTOR_SIZE / sizeof(uint32_t);

    for (index = 0U;index < word_count;++index)
    {
        if (word_pointer[index] != RECORD_STORAGE_ERASED_WORD)
        {
            return 0;
        }
    }

    return 1;
}
 
 static uint8_t RecordStorage_IsEntryValid(const RecordStorage_Entry_t *entry)
 {
    uint32_t calculated_crc;
     
     if(entry == NULL)
     {
        return 0;
     }
     if(entry->magic != RECORD_STORAGE_MAGIC)
     {
        return 0;
     }
     if(entry->verison != RECORD_STORAGE_VERSION)
     {
        return 0;
     }
     if ((entry->text_length == 0) || (entry->text_length > RECORD_STORAGE_TEXT_MAX_BYTES))
    {
        return 0;
    }

    if (entry->commit != RECORD_STORAGE_COMMIT)
    {
        return 0;
    }

    calculated_crc = RecordStorage_CalculateCrc32((const uint8_t *)entry,offsetof(RecordStorage_Entry_t,crc32));

    if (calculated_crc != entry->crc32)
    {
        return 0;
    }

    return 1;
 }
 

 /* ======================== 最近四条排序 ======================== */
 static void RecordStorage_InsertRecent(const RecordStorage_Entry_t *entry)
 {
    uint8_t insert_position;
     uint8_t move_index;
     uint16_t text_length;
     
     if(RecordStorage_IsEntryValid(entry) == 0)
     {
        return;
     }
     //防止同一序号被重复加入缓存
     for(insert_position = 0;insert_position < record_storage_recent_count;++insert_position)
     {
        if(record_storage_recent_sequence[insert_position] == entry->sequence)
        {
            return;
        }
     }
     //找到序号从新到旧的插入位置
     insert_position = 0;
     while(insert_position < record_storage_recent_count)
     {
        if(RecordStorage_SequenceIsNewer(entry->sequence,record_storage_recent_sequence[insert_position]) != 0)
        {
            break;
        }
        ++insert_position;
     }
     
     //已经有四条记录，并且当前记录比他们都旧
     if(insert_position >= RECORD_STORAGE_RECENT_MAX)
     {
        return;
     }
     if(record_storage_recent_count < RECORD_STORAGE_RECENT_MAX)
     {
        ++record_storage_recent_count;
     }
     
     //较旧的记录向后移动
     move_index = record_storage_recent_count - 1;
     while(move_index > insert_position)
     {
        record_storage_recent_sequence[move_index] = record_storage_recent_sequence[move_index - 1];
        record_storage_recent_length[move_index] = record_storage_recent_length[move_index - 1];
         (void)memcpy(record_storage_recent_text[move_index],
             record_storage_recent_text[move_index - 1],
             sizeof(record_storage_recent_text[move_index]));
         --move_index;
     }
     
     //插入当前记录
     text_length = entry->text_length;
     if(text_length > RECORD_STORAGE_TEXT_MAX_BYTES)
     {
        text_length = RECORD_STORAGE_TEXT_MAX_BYTES;
     }
     record_storage_recent_sequence[insert_position] = entry->sequence;
     record_storage_recent_length[insert_position] = text_length;
     (void)memset(record_storage_recent_text[insert_position],0,sizeof(record_storage_recent_text[insert_position]));
     (void)memcpy(record_storage_recent_text[insert_position],entry->text,text_length);
     record_storage_recent_text[insert_position][text_length] = '\0';
 }
 
 
 /* ======================== 扇区扫描 ======================== */
 static void RecordStorage_ScanSector(uint8_t sector_index)
 {
    RecordStorage_SectorState_t *state;
    const RecordStorage_Entry_t *entry;
     
     uint32_t slot_index;
     uint32_t address;
     
     if(sector_index >= 2)
     {
        return;
     }
     
     state = &record_storage_sector[sector_index];
     state->first_erased_address = 0L;
     state->maxmum_sequence = 0L;
     state->valid_count = 0L;
     state->has_valid_record = 0;
     
     for(slot_index = 0;slot_index < (uint32_t)RECORD_STORAGE_SLOT_COUNT;++slot_index)
     {
        address = state->base_address + slot_index * sizeof(RecordStorage_Entry_t);
         //保存第一个完全未写入的地址
         if(RecordStorage_IsSlotErased(address) != 0)
         {
            if(state->first_erased_address == 0L)
            {
                state->first_erased_address = address;
            }
            continue;
         }
         
         entry = (const RecordStorage_Entry_t *)address;
         
         //忽略CRC错误
         if(RecordStorage_IsEntryValid(entry) == 0)
         {
            continue;
         }
         
         ++state->valid_count;
         
         if((state->has_valid_record == 0) || (RecordStorage_SequenceIsNewer(entry->sequence,state->maxmum_sequence) != 0))
         {
            state->maxmum_sequence = entry->sequence;
             state->has_valid_record = 1;
         }
         RecordStorage_InsertRecent(entry);
     }
 }
 
 
 static uint32_t RecordStorage_FindFirstErasedSlotFrom(uint32_t sector_base,uint32_t start_address)
 {
    uint32_t sector_end;
     uint32_t address;
     
     sector_end = sector_base + RECORD_STORAGE_SECTOR_SIZE;
     if(start_address < sector_base)
     {
        start_address = sector_base;
     }
     address = start_address;
     
     while((address + sizeof(RecordStorage_Entry_t)) <= sector_end)
     {
        if(RecordStorage_IsSlotErased(address) != 0)
        {
            return address;
        }
        address += sizeof(RecordStorage_Entry_t);
     }
     return 0;
 }
 
 
 
 /* ======================== 看门狗临时延时 ======================== */

/*
 * 当前工程看门狗超时时间约0.5秒，
 * 但擦除128 KB扇区可能超过该时间。
 *
 * 擦除前临时把IWDG预分频提高到256，
 * 擦除完成后恢复原配置。
 *
 * 注意：
 * Flash擦除期间CPU实时任务仍会暂停。
 * 必须确保擦除时发射和调平电机已经停止。
 */
 static uint8_t RecordStorage_ExtendWatchdog(IWDG_InitTypeDef *saved_configuration)
 {
    if(saved_configuration == NULL)
    {
        return 0;
    }
    *saved_configuration = hiwdg.Init;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload = 4095;
    
    if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
    hiwdg.Init = *saved_configuration;
        return 0;
    }
    (void)HAL_IWDG_Refresh(&hiwdg);
    return 1;
 }
 
 
 static uint8_t RecordStorage_RestoreWatchdog(const IWDG_InitTypeDef *saved_configuration)
 {
    if(saved_configuration == NULL)
    {
        return 0;
    }
    
    hiwdg.Init = *saved_configuration;
    
    if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        return 0;
    }
    (void)HAL_IWDG_Refresh(&hiwdg);
    return 1;
 }
 
 
 
 /* ======================== Flash擦除 ======================== */
 static void RecordStorage_ResetFlashDataCache(void)
 {
    uint32_t data_cache_enabled;
     data_cache_enabled = FLASH->ACR & FLASH_ACR_DCEN;
     
     if(data_cache_enabled != 0)
     {
        __HAL_FLASH_DATA_CACHE_DISABLE();
         __HAL_FLASH_DATA_CACHE_RESET();
         __HAL_FLASH_DATA_CACHE_ENABLE();
     }
 }
 
 
 static uint8_t RecordStorage_EraseSector(uint8_t sector_index)
{
    FLASH_EraseInitTypeDef erase_configuration;
    IWDG_InitTypeDef saved_watchdog_configuration;

    HAL_StatusTypeDef erase_status;
    uint32_t sector_error;
    uint8_t watchdog_extended;
    uint8_t watchdog_restored;

    if (sector_index >= 2)
    {
        return 0;
    }

    //已经完全擦除时，不重复擦除，
    if (RecordStorage_IsSectorErased(record_storage_sector[sector_index].base_address) != 0)
    {
        return 1;
    }

    //防止擦除期间触发0.5秒看门狗复位。
    watchdog_extended = RecordStorage_ExtendWatchdog(&saved_watchdog_configuration);

    if (watchdog_extended == 0)
    {
        return 0;
    }

    (void)HAL_IWDG_Refresh(&hiwdg);

    erase_configuration.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_configuration.Sector = record_storage_sector[sector_index].sector_number;
    erase_configuration.NbSectors = 1;
    erase_configuration.VoltageRange = RECORD_STORAGE_VOLTAGE_RANGE;

    sector_error = 0xFFFFFFFFL;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        (void)RecordStorage_RestoreWatchdog(&saved_watchdog_configuration);
        return 0U;
    }

    //清除上一次Flash操作留下的状态标志。
        __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP |
        FLASH_FLAG_OPERR |
        FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR |
        FLASH_FLAG_PGPERR |
        FLASH_FLAG_PGSERR);

    erase_status = HAL_FLASHEx_Erase(&erase_configuration,&sector_error);
    (void)HAL_FLASH_Lock();
    RecordStorage_ResetFlashDataCache();
    watchdog_restored = RecordStorage_RestoreWatchdog(&saved_watchdog_configuration);

    if ((erase_status != HAL_OK) || (sector_error != 0xFFFFFFFFUL) || (watchdog_restored == 0))
    {
        return 0;
    }

    // 重新读取整个扇区，确认擦除完成
    if (RecordStorage_IsSectorErased(record_storage_sector[sector_index].base_address) == 0)
    {
        return 0;
    }

    record_storage_sector[sector_index].first_erased_address = record_storage_sector[sector_index].base_address;

    record_storage_sector[sector_index].maxmum_sequence = 0L;

    record_storage_sector[sector_index].valid_count = 0L;

    record_storage_sector[sector_index].has_valid_record = 0U;

    return 1;
}



/* ======================== Flash写入 ======================== */
static uint8_t RecordStorage_ProgramEntry(uint32_t address,const RecordStorage_Entry_t *source)
{
    const uint32_t *source_word;
    const RecordStorage_Entry_t *flash_entry;
    
    HAL_StatusTypeDef program_status;
    
    uint32_t word_index;
    uint32_t words_before_commit;
    uint32_t commit_address;
    
    if(source == NULL)
    {
        return 0;
    }
    if((address & 3L) != 0)
    {
        return 0;
    }
    if(RecordStorage_IsSlotErased(address) == 0)
    {
        return 0;
    }
    source_word = (const uint32_t *)source;
    words_before_commit = (uint32_t)(offsetof(RecordStorage_Entry_t,commit) / sizeof(uint32_t));
    commit_address = address + offsetof(RecordStorage_Entry_t,commit);
    
    (void)HAL_IWDG_Refresh(&hiwdg);
    
    if(HAL_FLASH_Unlock() != HAL_OK)
    {
        return 0;
    }
    
     __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP |
        FLASH_FLAG_OPERR |
        FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR |
        FLASH_FLAG_PGPERR |
        FLASH_FLAG_PGSERR);

    program_status = HAL_OK;
    
    //先写入commit之前的全部数据。
    for (word_index = 0L;word_index < words_before_commit;++word_index)
    {
        program_status =HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address + word_index * sizeof(uint32_t),source_word[word_index]);

        if (program_status != HAL_OK)
        {
            break;
        }
    }
    
    //所有数据写完后，最后写入commit。
     if (program_status == HAL_OK)
    {
        program_status =HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,commit_address,RECORD_STORAGE_COMMIT);
    }

    (void)HAL_FLASH_Lock();
    RecordStorage_ResetFlashDataCache();
    (void)HAL_IWDG_Refresh(&hiwdg);
    if (program_status != HAL_OK)
    {
        return 0;
    }
    
    // 写入后重新进行完整CRC校验。
    flash_entry =(const RecordStorage_Entry_t *)address;

    return RecordStorage_IsEntryValid(flash_entry);
    
}


/* ======================== 初始化 ======================== */

uint8_t RecordStorage_Init(void)
{
    uint32_t newest_sequence;

    (void)memset(record_storage_sector,0,sizeof(record_storage_sector));
    (void)memset(record_storage_recent_text,0,sizeof(record_storage_recent_text));
    (void)memset(record_storage_recent_length,0,sizeof(record_storage_recent_length));
    (void)memset(record_storage_recent_sequence,0,sizeof(record_storage_recent_sequence));

    record_storage_recent_count = 0;
    record_storage_initialized = 0;

    //配置Sector 10
    record_storage_sector[0].base_address = RECORD_STORAGE_SECTOR_A_BASE;
    record_storage_sector[0].sector_number = RECORD_STORAGE_SECTOR_A_NUMBER;

    //配置Sector 11
    record_storage_sector[1].base_address = RECORD_STORAGE_SECTOR_B_BASE;
    record_storage_sector[1].sector_number = RECORD_STORAGE_SECTOR_B_NUMBER;

    //扫描两个扇区并恢复最近四条记录
    RecordStorage_ScanSector(0);
    RecordStorage_ScanSector(1);

    //根据两个扇区的最大序号选择活动扇区
    if ((record_storage_sector[0].has_valid_record != 0) && (record_storage_sector[1].has_valid_record != 0))
    {
        if (RecordStorage_SequenceIsNewer(record_storage_sector[1].maxmum_sequence,record_storage_sector[0].maxmum_sequence) != 0)
        {
            record_storage_active_sector = 1;
        }
        else
        {
            record_storage_active_sector = 0;
        }
    }
    else if (record_storage_sector[1].has_valid_record != 0)
    {
        record_storage_active_sector = 1;
    }
    else
    {
        //两个扇区均为空，或者只有Sector 10有记录
        record_storage_active_sector = 0;
    }

    record_storage_next_address = record_storage_sector[record_storage_active_sector].first_erased_address;

    //最近记录数组已经按照从新到旧排列
    if (record_storage_recent_count != 0)
    {
        newest_sequence = record_storage_recent_sequence[0];
        record_storage_next_sequence = RecordStorage_GetNextSequence(newest_sequence);
    }
    else
    {
        record_storage_next_sequence = 1L;
    }

    record_storage_initialized = 1;
    return 1;
}

/* ======================== 增加记录 ======================== */

uint8_t RecordStorage_Append(const uint8_t *text,uint16_t text_length)
{
    RecordStorage_Entry_t entry;

    uint8_t write_sector;
    uint8_t other_sector;
    uint32_t write_address;
    uint32_t next_search_address;

    if (record_storage_initialized == 0)
    {
        return 0;
    }

    if ((text == NULL) || (text_length == 0) || (text_length > RECORD_STORAGE_TEXT_MAX_BYTES))
    {
        return 0;
    }

    write_sector = record_storage_active_sector;
    write_address = record_storage_next_address;

    //当前活动扇区已满，切换到另一个扇区
    if (write_address == 0L)
    {
        other_sector = (write_sector == 0) ? 1 : 0;

        /*
         * 另一个扇区如果包含旧记录，
         * 需要先擦除。
         *
         * 注意：擦除可能暂停CPU约1～2秒。
         * 调用到这里时必须保证所有电机已经停止。
         */
        if (RecordStorage_EraseSector(other_sector) == 0)
        {
            return 0;
        }

        write_sector = other_sector;
        write_address = record_storage_sector[write_sector].base_address;
    }

    /*
     * 创建待写入记录。
     *
     * 未使用区域保持0xFF，
     * 与Flash擦除状态一致。
     */
    (void)memset(&entry,0xFF,sizeof(entry));

    entry.magic = RECORD_STORAGE_MAGIC;
    entry.sequence = record_storage_next_sequence;
    entry.text_length = text_length;
    entry.verison = RECORD_STORAGE_VERSION;

    (void)memcpy(entry.text,text,text_length);

    //text未占满128字节时增加字符串结束符
    if (text_length < RECORD_STORAGE_TEXT_MAX_BYTES)
    {
        entry.text[text_length] = 0;
    }

    //CRC不包含crc32和commit字段
    entry.crc32 = RecordStorage_CalculateCrc32((const uint8_t *)&entry,offsetof(RecordStorage_Entry_t,crc32));

    /*
     * RAM结构中的commit保持0xFFFFFFFF，
     * RecordStorage_ProgramEntry会最后单独写入。
     */
    entry.commit = RECORD_STORAGE_ERASED_WORD;

    if (RecordStorage_ProgramEntry(write_address,&entry) == 0)
    {
        /*
         * 当前槽可能已经被部分写入。
         * 后续不能再次覆盖它，需要寻找下一个空槽。
         */
        if (write_sector == record_storage_active_sector)
        {
            record_storage_next_address =
                RecordStorage_FindFirstErasedSlotFrom(record_storage_sector[write_sector].base_address,
                    write_address +sizeof(RecordStorage_Entry_t));
        }
        return 0;
    }

    //更新当前活动扇区状态
    record_storage_active_sector = write_sector;
    record_storage_sector[write_sector].has_valid_record = 1;
    ++record_storage_sector[write_sector].valid_count;
    record_storage_sector[write_sector].maxmum_sequence =entry.sequence;

    //更新最近四条RAM缓存
    entry.commit = RECORD_STORAGE_COMMIT;
    RecordStorage_InsertRecent(&entry);

    //寻找下一个空槽
    next_search_address = write_address + sizeof(RecordStorage_Entry_t);
    record_storage_next_address = RecordStorage_FindFirstErasedSlotFrom(record_storage_sector[write_sector].base_address,next_search_address);
    record_storage_sector[write_sector].first_erased_address = record_storage_next_address;

    //更新下一条记录序号
    record_storage_next_sequence = RecordStorage_GetNextSequence(entry.sequence);

    return 1;
}

uint8_t RecordStorage_NeedsErase(void)
{
    if (record_storage_initialized == 0U)
    {
        return 0U;
    }

    return (record_storage_next_address == 0U) ? 1U : 0U;
}

/* ======================== 读取记录 ======================== */

uint8_t RecordStorage_GetRecentCount(void)
{
    if (record_storage_initialized == 0)
    {
        return 0;
    }

    return record_storage_recent_count;
}

uint8_t RecordStorage_ReadRecent(uint8_t recent_index,char *destination,size_t destination_size)
{
    size_t copy_length;

    if (record_storage_initialized == 0)
    {
        return 0;
    }

    if ((destination == NULL) || (destination_size == 0))
    {
        return 0;
    }

    if (recent_index >= record_storage_recent_count)
    {
        destination[0] = '\0';
        return 0;
    }

    copy_length = record_storage_recent_length[recent_index];

    //为字符串结束符保留一个字节
    if (copy_length >= destination_size)
    {
        copy_length = destination_size - 1;
    }

    (void)memcpy(destination,record_storage_recent_text[recent_index],copy_length);
    destination[copy_length] = '\0';

    return 1;
}
