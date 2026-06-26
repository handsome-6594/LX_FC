#include <string.h>
#include "fifo.h"
#include "main.h"
 
/********************************************************************//**
 * @brief       FIFO初始化
 * @param[in]   pFIFO: FIFO指针
 * @param[in]	pBuff: FIFO中缓存
 * @param[in]	elementBytes:FIFO每个元素的字节数
 * @param[in]	depth: FIFO深度
 * @return      None
 *********************************************************************/
void FIFO_Init(FIFO_Type *pFIFO, void *pBuff, unsigned int elementBytes, unsigned int depth)
{
	pFIFO->Buff = pBuff;
	pFIFO->ElementBytes = elementBytes;
	pFIFO->Depth = depth;
	pFIFO->Head = 0;
	pFIFO->Tail = 0;
	pFIFO->Counter = 0;
}
 
/********************************************************************//**
 * @brief       判断FIFO是否为空
 * @param[in]   pFIFO: FIFO指针
 * @return      1-TRUE or 0-FALSE
 *********************************************************************/
unsigned char FIFO_IsEmpty(FIFO_Type *pFIFO)
{
	return (pFIFO->Counter == 0);
}
 
/********************************************************************//**
 * @brief       判断FIFO是否已满
 * @param[in]   pFIFO: FIFO指针
 * @return      TRUE or FALSE
 *********************************************************************/
unsigned char FIFO_IsFull(FIFO_Type *pFIFO)
{
	return (pFIFO->Counter == pFIFO->Depth);
}
 
/********************************************************************//**
 * @brief       向FIFO添加一个元素
 * @param[in]   pFIFO: FIFO指针
 * @param[in]	pValue: 要添加的元素
 * @return      1-TRUE or 0-FALSE
 *********************************************************************/
unsigned char FIFO_AddOne(FIFO_Type *pFIFO, void *pValue)
{
	unsigned char *p;
 
	if (FIFO_IsFull(pFIFO))
	{
		return 0;
	}
 
	p = (unsigned char *)pFIFO->Buff;
	memcpy(p + pFIFO->Tail * pFIFO->ElementBytes, (unsigned char *)pValue, pFIFO->ElementBytes);
	
	pFIFO->Tail ++;
	if (pFIFO->Tail >= pFIFO->Depth)
	{
		pFIFO->Tail = 0;
	}
	pFIFO->Counter ++;
	return 1;
}
 
/********************************************************************//**
 * @brief       向FIFO添加叶喔鲻元素
 * @param[in]   pFIFO: FIFO指针
 * @param[in]	pValues: 要添加的元素指针
 * @param[in]	bytesToAdd:要添加元素的长度
 * @return     实际添加的元素个数
 *********************************************************************/
unsigned int FIFO_Add(FIFO_Type *pFIFO, void *pValues, unsigned int bytesToAdd)
{
	unsigned char *p;
	unsigned int cnt = 0;
 
	p = (unsigned char *)pValues;
	while(bytesToAdd --)
	{
		if (FIFO_AddOne(pFIFO, p))
		{
			p += pFIFO->ElementBytes;
			cnt++;
		}
		else
		{
			break;
		}
	}
 
	return cnt;
}
 
/********************************************************************//**
 * @brief       从FIFO读取一个元素
 * @param[in]   pFIFO: FIFO指针
 * @param[in]	pValue: 存放要读取的元素指针
 * @return      1-TRUE or 0-FALSE
 *********************************************************************/
unsigned char FIFO_GetOne(FIFO_Type *pFIFO, void *pValue)
{
	unsigned char *p;
	if (FIFO_IsEmpty(pFIFO))
	{
		return 0;
	}
 
	p = (unsigned char *)pFIFO->Buff;
	memcpy(pValue, p + pFIFO->Head * pFIFO->ElementBytes, pFIFO->ElementBytes);
 
	pFIFO->Head ++;
	if (pFIFO->Head >= pFIFO->Depth)
	{
		pFIFO->Head = 0;
	}
	pFIFO->Counter --;
 
	return 1;
}
 
/********************************************************************//**
 * @brief       从FIFO读取多个元素
 * @param[in]   pFIFO: FIFO指针
 * @param[out]	pValues: 存放要读取的元素指针
 * @param[in]	bytesToRead: 尧都区的元素长度
 * @return     实际读取的元素个数
 *********************************************************************/
unsigned int FIFO_Get(FIFO_Type *pFIFO, void *pValues, unsigned int bytesToRead)
{
	unsigned int cnt = 0;
	unsigned char *p;
 
	p = pValues;
	while(bytesToRead--)
	{
		if (FIFO_GetOne(pFIFO, p))
		{
			p += pFIFO->ElementBytes;
			cnt++;
		}
		else
		{
			break;
		}
	}
 
	return cnt;
}
 
/********************************************************************//**
 * @brief       清空FIFO
 * @param[in]   pFIFO: FIFO指针
 * @return      None
 *********************************************************************/
void FIFO_Clear(FIFO_Type *pFIFO)
{
	pFIFO->Counter = 0;
	pFIFO->Head = 0;
	pFIFO->Tail = 0;
} 
