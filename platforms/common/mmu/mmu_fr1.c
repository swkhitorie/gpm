#include "mmu_fr.h"

/*
* Initialises the heap structures before their first use.
*/
static uint8_t *ucHeap;

static size_t configTOTAL_HEAP_SIZE;

static BaseType_t xHeapHasBeenInitialised = pdFALSE;

#define configADJUSTED_HEAP_SIZE	( configTOTAL_HEAP_SIZE - portBYTE_ALIGNMENT )

/* Define the linked list structure.  This is used to link free blocks in order
of their size. */
typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
    size_t xBlockSize;						/*<< The size of the free block. */
} BlockLink_t;

static const uint16_t heapSTRUCT_SIZE	= ( ( sizeof ( BlockLink_t ) + ( portBYTE_ALIGNMENT - 1 ) ) & ~portBYTE_ALIGNMENT_MASK );
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( heapSTRUCT_SIZE * 2 ) )

/* Create a couple of list links to mark the start and end of the list. */
static BlockLink_t xStart, xEnd;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t xFreeBytesRemaining = configADJUSTED_HEAP_SIZE;

/* STATIC FUNCTIONS ARE DEFINED AS MACROS TO MINIMIZE THE FUNCTION CALL DEPTH. */

/*
* Insert a block into the list of free blocks - which is ordered by size of
* the block.  Small blocks at the start of the list and large blocks at the end
* of the list.
*/
#define prvInsertBlockIntoFreeList( pxBlockToInsert )								\
{																					\
BlockLink_t *pxIterator;															\
size_t xBlockSize;																	\
                                                                                    \
    xBlockSize = pxBlockToInsert->xBlockSize;										\
                                                                                    \
    /* Iterate through the list until a block is found that has a larger size */	\
    /* than the block we are inserting. */											\
    for( pxIterator = &xStart; pxIterator->pxNextFreeBlock->xBlockSize < xBlockSize; pxIterator = pxIterator->pxNextFreeBlock )	\
    {																				\
        /* There is nothing to do here - just iterate to the correct position. */	\
    }																				\
                                                                                    \
    /* Update the list to include the block being inserted in the correct */		\
    /* position. */																	\
    pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;					\
    pxIterator->pxNextFreeBlock = pxBlockToInsert;									\
}
/*-----------------------------------------------------------*/

void *rtos_mem1_malloc( size_t xWantedSize )
{
    BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
    void *pvReturn = NULL;

    /* If this is the first call to malloc then the heap will require
    initialisation to setup the list of free blocks. */
    if( xHeapHasBeenInitialised == pdFALSE )
    {
        return NULL;
    }

    /* The wanted size is increased so it can contain a BlockLink_t
    structure in addition to the requested amount of bytes. */
    if( xWantedSize > 0 )
    {
        xWantedSize += heapSTRUCT_SIZE;

        /* Ensure that blocks are always aligned to the required number of bytes. */
        if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0 )
        {
            /* Byte alignment required. */
            xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
        }
    }

    if( ( xWantedSize > 0 ) && ( xWantedSize < configADJUSTED_HEAP_SIZE ) )
    {
        /* Blocks are stored in byte order - traverse the list from the start
        (smallest) block until one of adequate size is found. */
        pxPreviousBlock = &xStart;
        pxBlock = xStart.pxNextFreeBlock;
        while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
        {
            pxPreviousBlock = pxBlock;
            pxBlock = pxBlock->pxNextFreeBlock;
        }

        /* If we found the end marker then a block of adequate size was not found. */
        if( pxBlock != &xEnd )
        {
            /* Return the memory space - jumping over the BlockLink_t structure
            at its start. */
            pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + heapSTRUCT_SIZE );

            /* This block is being returned for use so must be taken out of the
            list of free blocks. */
            pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

            /* If the block is larger than required it can be split into two. */
            if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
            {
                /* This block is to be split into two.  Create a new block
                following the number of bytes requested. The void cast is
                used to prevent byte alignment warnings from the compiler. */
                pxNewBlockLink = ( BlockLink_t * ) ( ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize ) );

                /* Calculate the sizes of two blocks split from the single
                block. */
                pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
                pxBlock->xBlockSize = xWantedSize;

                /* Insert the new block into the list of free blocks. */
                prvInsertBlockIntoFreeList( ( pxNewBlockLink ) );
            }

            xFreeBytesRemaining -= pxBlock->xBlockSize;
        }
    }

    return pvReturn;
}
/*-----------------------------------------------------------*/

void rtos_mem1_free( void *pv )
{
    uint8_t *puc = ( uint8_t * ) pv;
    BlockLink_t *pxLink;

    if( pv != NULL )
    {
        /* The memory being freed will have an BlockLink_t structure immediately
        before it. */
        puc -= heapSTRUCT_SIZE;

        /* This unexpected casting is to keep some compilers from issuing
        byte alignment warnings. */
        pxLink = ( BlockLink_t * ) ( ( void * ) puc );

        /* Add this block to the list of free blocks. */
        prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
        xFreeBytesRemaining += pxLink->xBlockSize;
    }
}
/*-----------------------------------------------------------*/

size_t rtos_mem1_get_freeheapsize( void )
{
    return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void rtos_mem1_heap_init( void *mem, size_t size )
{
    BlockLink_t *pxFirstFreeBlock;
    uint8_t *pucAlignedHeap;

    ucHeap = ( uint8_t * ) mem;

    configTOTAL_HEAP_SIZE = size;

    xHeapHasBeenInitialised = pdTRUE;

    /* Ensure the heap starts on a correctly aligned boundary. */
    pucAlignedHeap = ( uint8_t * ) ( ( ( portPOINTER_SIZE_TYPE ) &ucHeap[ portBYTE_ALIGNMENT ] ) & ( ~( ( portPOINTER_SIZE_TYPE ) portBYTE_ALIGNMENT_MASK ) ) );

    /* xStart is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
    xStart.pxNextFreeBlock = ( BlockLink_t * ) ( ( void * ) pucAlignedHeap );
    xStart.xBlockSize = ( size_t ) 0;

    /* xEnd is used to mark the end of the list of free blocks. */
    xEnd.xBlockSize = configADJUSTED_HEAP_SIZE;
    xEnd.pxNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
    entire heap space. */
    pxFirstFreeBlock = ( BlockLink_t * ) ( ( void * ) pucAlignedHeap );
    pxFirstFreeBlock->xBlockSize = configADJUSTED_HEAP_SIZE;
    pxFirstFreeBlock->pxNextFreeBlock = &xEnd;
}
/*-----------------------------------------------------------*/


