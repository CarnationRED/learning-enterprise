#include "FIFO.h"
#include <string.h>
 
/**
* @brief fifo结构
*/
 
typedef struct
{
    int ptr_write;//写入量
    int ptr_read;//读取量
    bool is_full;//FIFO状态
 
    int item_sum;// fifo中存储的元素数,不是字节大小
    int item_size;// 元素大小.单位: 字节
    void *fifo_ptr; //FIFO地址
} Fifo;
 
/**
* @brief 创建fifo
* @param item_sum：fifo中元素数.注意不是字节数
* @param item_size: 元素大小.单位: 字节
* @return fifo索引
*/
 
int fifo_create(int item_sum, int item_size)
{
    Fifo *fifo = (Fifo *)malloc(sizeof(Fifo));
    fifo->item_sum = item_sum;
    fifo->item_size = item_size;
    fifo->ptr_write = 0;
    fifo->ptr_read = 0;
    fifo->is_full = false;
    fifo->fifo_ptr = (void *)malloc(item_sum * item_size);
    return (int)fifo;
}
 
/**
* @brief 删除fifo
* @param fifo_index: fifo索引
*/
 
void fifo_delete(int fifo_index)
{
    Fifo *fifo = (Fifo *)fifo_index;
    free(fifo->fifo_ptr);
    fifo->fifo_ptr = NULL;
    free(fifo);
    fifo = NULL;
}
 
/**
* @brief fifo检查是否可以写入
* @param fifo_index: fifo索引
* @retval false:不可以写入.true:可以写入
*/
 
bool fifo_writeable(int fifo_index)
{	
    Fifo *fifo = (Fifo *)fifo_index;
    return !fifo->is_full;
}
 
/**
* @brief fifo写入
* @param fifo_index: fifo索引
* @param frame:写入元素指针
* @return false:失败.true:成功
*/
 
bool fifo_write(int fifo_index, void *data)
{
  Fifo *fifo = (Fifo *)fifo_index;
  if (fifo->is_full)//是否能写入
	{
		return false;
	}
	
  memcpy((char *)(fifo->fifo_ptr) + fifo->ptr_write * fifo->item_size, data, fifo->item_size);//将数据复制到FIFO中
  fifo->ptr_write++;//写入量+1
 
  if (fifo->ptr_write >= fifo->item_sum)
  {
    fifo->is_full = true;//FIFO满
  }
	
	return true;
}
 
/**
* @brief fifo批量写入
* @param fifo_index: fifo索引
* @param data: 写入元素指针
* @param item_num：写入的元素数目
* @return false:失败.true:成功
*/
 
bool fifo_write_batch(int fifo_index, void *data, int item_num)
{
	Fifo *fifo = (Fifo *)fifo_index;
	if (fifo_writeable_item_count((int)fifo) < item_num)
	{
		return false;
	}
 
	memcpy((char *)(fifo->fifo_ptr) + fifo->ptr_write * fifo->item_size, data, fifo->item_size * item_num);
	fifo->ptr_write += item_num;
	if (fifo->ptr_write >= fifo->item_sum)
	{
		fifo->is_full = true;//FIFO满
	}
  return true;
}
 
/**
* @brief fifo检查是否可以读取
* @param fifo_index: fifo索引
* @return false:不可以读取.true:可以读取
*/
 
bool fifo_readable(int fifo_index)
{
  Fifo *fifo = (Fifo *)fifo_index;
  if (fifo->ptr_write == 0)//写入量为0 不可读取
	{
		return false;
	}
	return true;
}
 
/**
* @brief fifo读取
* @param fifo_index: fifo索引
* @param data: 读取的数据
* @return false: 失败.true: 成功
*/
 
bool fifo_read(int fifo_index, void *data)
{
  Fifo *fifo = (Fifo *)fifo_index;
  if (fifo->ptr_write == fifo->ptr_read && !fifo->is_full)
	{
		return false;
	}
	memcpy(data, (char *)(fifo->fifo_ptr), fifo->item_size);
	fifo->ptr_read ++;
	//-----------------------------------------//
	memcpy((char *)(fifo->fifo_ptr),(char *)(fifo->fifo_ptr) + fifo->ptr_read * fifo->item_size,(fifo->item_sum - fifo->ptr_read));//整体数据左对齐
	fifo->ptr_write -= fifo->ptr_read;//写数据量-=读出的数据量
	fifo->ptr_read = 0;//读出量清零
	//========================================//
	fifo->is_full = false;
	return true;
}
 
/**
* @brief fifo批量读取
* @param fifo_index: fifo索引
* @param data: 读取的数据
* @param item_num：读取的元素数
* @return false: 失败.true: 成功
*/
 
bool fifo_read_batch(int fifo_index, void *data, int item_num)
{
	Fifo *fifo = (Fifo *)fifo_index;
	if (fifo_readable_item_count((int)fifo) < item_num)
	{
		return false;
	}
	memcpy(data, (char *)(fifo->fifo_ptr), fifo->item_size * item_num);
	fifo->ptr_read += item_num;
	//-----------------------------------------//
	// memcpy((char *)(fifo->fifo_ptr),(char *)(fifo->fifo_ptr) + fifo->ptr_read * fifo->item_size,(fifo->item_sum - fifo->ptr_read));//整体数据左对齐
       memcpy((char *)(fifo->fifo_ptr),(char *)(fifo->fifo_ptr) + fifo->ptr_read * fifo->item_size,(fifo->item_sum - fifo->ptr_read)*fifo->item_size);//cited:https://blog.csdn.net/qq_24025329/article/details/122970830
	fifo->ptr_write -= fifo->ptr_read;//写数据量-=读出的数据量
	fifo->ptr_read = 0;//读出量清零
	//========================================//
	fifo->is_full = false;
	return true;
}
 
/**
* @brief fifo可读的元素数
* @param fifo_index: fifo索引
* @return 元素数
*/
 
int fifo_readable_item_count(int fifo_index)
{
	Fifo *fifo = (Fifo *)fifo_index;
	if (fifo->is_full)
	{
		return fifo->item_sum;
	}
	else
	{
		return fifo->ptr_write;
	}
}
 
/**
* @brief fifo可写的元素数
* @param fifo_index: fifo索引
* @return 元素数
*/
 
int fifo_writeable_item_count(int fifo_index)
{
	Fifo *fifo = (Fifo *)fifo_index;
	if (fifo->is_full)
	{
		return 0;
	}
	else
	{
		return fifo->item_sum - fifo->ptr_write;
	}
}
// int FIFO1;
// int Count;
// bool error;
// unsigned char linshi[] = "lingshishuju";
// unsigned char ceshi[]="1234567890";
// unsigned char g_cEE[11];
// void TestFifo()
// {
// 	FIFO1 = fifo_create(10,1);//创建FIFO 深度10 字节1
// 	error = fifo_writeable(FIFO1);//是否可以写入
// 	error = fifo_readable(FIFO1);//是否可读
// 	Count = fifo_writeable_item_count(FIFO1);//可写入的数据量
// 	fifo_write(FIFO1, ceshi);//写入数据
// 	fifo_write(FIFO1, &ceshi[1]);//写入数据
// 	error = fifo_readable(FIFO1);//是否可读
// 	Count = fifo_readable_item_count(FIFO1);//可读数据量
// 	fifo_read(FIFO1,g_cEE);//读
// 	fifo_write(FIFO1, &ceshi[2]);//写入数据
// 	fifo_read(FIFO1,&g_cEE[1]);//读
// 	fifo_read(FIFO1,&g_cEE[2]);//读
// 	//批量
// 	Count = fifo_writeable_item_count(FIFO1);//可写入的数据量
// 	error = fifo_write_batch(FIFO1,linshi,Count);//批量写入
// 	error = fifo_read_batch(FIFO1,g_cEE,3);//批量读
// 	error = fifo_writeable(FIFO1);//是否可以写入
// 	Count = fifo_writeable_item_count(FIFO1);//可写入的数据量
// 	error = fifo_write_batch(FIFO1,linshi,Count);//批量写入
// 	Count = fifo_readable_item_count(FIFO1);//可读数据量
// 	error = fifo_read_batch(FIFO1,g_cEE,Count);//批量读
// 	error = fifo_writeable(FIFO1);//是否可以写入
// 	error = fifo_readable(FIFO1);//是否可读
	
// }