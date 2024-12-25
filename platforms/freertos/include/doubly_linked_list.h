#ifndef DOUBLY_LINKED_LIST_H_
#define DOUBLY_LINKED_LIST_H_

#include <stddef.h>
#include <stdint.h>

typedef struct Link
{
    struct Link *pxPrev;
    struct Link *pxNext;
} Link_t;

#define listINIT_HEAD(pxHead)        \
    {                                \
        (pxHead)->pxPrev = (pxHead); \
        (pxHead)->pxNext = (pxHead); \
    }

#define listADD(pxHead, pxLink)                  \
    {                                            \
        Link_t *pxPrevLink = (pxHead);           \
        Link_t *pxNextLink = ((pxHead)->pxNext); \
                                                \
        (pxLink)->pxNext = pxNextLink;           \
        pxNextLink->pxPrev = (pxLink);           \
        pxPrevLink->pxNext = (pxLink);           \
        (pxLink)->pxPrev = (pxPrevLink);         \
    }

#define listREMOVE(pxLink)                                         \
    {                                                              \
        if (( pxLink)->pxNext != NULL && (pxLink)->pxPrev != NULL) \
        {                                                          \
            (pxLink)->pxPrev->pxNext = (pxLink)->pxNext;           \
            (pxLink)->pxNext->pxPrev = (pxLink)->pxPrev;           \
        }                                                          \
        (pxLink)->pxPrev = NULL;                                   \
        (pxLink)->pxNext = NULL;                                   \
    }

#define listIS_EMPTY(pxHead)    (((pxHead) == NULL) || ((pxHead)->pxNext == (pxHead)))

#define listPOP(pxHead, pxLink)                                         \
    {                                                                   \
        if (listIS_EMPTY((pxHead))) {                                   \
            (pxLink) = NULL;                                            \
        } else {                                                        \
            (pxLink) = (pxHead)->pxNext;                                \
            if ((pxLink)->pxNext != NULL && (pxLink)->pxPrev != NULL) { \
                (pxLink)->pxPrev->pxNext = (pxLink)->pxNext;            \
                (pxLink)->pxNext->pxPrev = (pxLink)->pxPrev;            \
            }                                                           \
            (pxLink)->pxPrev = NULL;                                    \
            (pxLink)->pxNext = NULL;                                    \
        }                                                               \
    }

#define listMERGE(pxHeadResultList, pxHeadListToMerge)                         \
    {                                                                          \
        if (!listIS_EMPTY((pxHeadListToMerge))) {                              \
            (pxHeadListToMerge)->pxPrev->pxNext = (pxHeadResultList)->pxNext;  \
            (pxHeadResultList)->pxNext->pxPrev = (pxHeadListToMerge)->pxPrev;  \
            (pxHeadListToMerge)->pxNext->pxPrev = (pxHeadResultList);          \
            (pxHeadResultList)->pxNext = (pxHeadListToMerge)->pxNext;          \
            listINIT_HEAD((pxHeadListToMerge));                                \
        }                                                                      \
    }

#define listFOR_EACH(pxLink, pxHead)  \
    for ((pxLink) = (pxHead)->pxNext; \
        (pxLink) != (pxHead);         \
        (pxLink) = (pxLink)->pxNext)

#define listFOR_EACH_SAFE(pxLink, pxTempLink, pxHead)                  \
    for ((pxLink) = (pxHead)->pxNext, (pxTempLink) = (pxLink)->pxNext; \
        (pxLink) != (pxHead);                                          \
        (pxLink) = (pxTempLink), (pxTempLink) = (pxLink)->pxNext )

#define listCONTAINER(pxLink, type, member)    ((type *)((uint8_t *)(pxLink) - (uint8_t *) (&((type*) 0)->member)))

#endif
