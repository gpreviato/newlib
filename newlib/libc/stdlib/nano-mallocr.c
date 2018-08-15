/*
 * Copyright (c) 2012, 2013 ARM Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the company may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ARM LTD ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ARM LTD BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Implementation of <<malloc>> <<free>> <<calloc>> <<realloc>>, optional
 * as to be reenterable.
 *
 * Interface documentation refer to malloc.c.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <malloc.h>

//#define _NANOMALLOC_FIRST_FREE 1
#define _NANOMALLOC_BEST_FIT 1


#if DEBUG
#include <assert.h>
#else
#define assert(x) ((void)0)
#endif

#ifndef MAX
#define MAX(a,b) ((a) >= (b) ? (a) : (b))
#endif

#define _SBRK_R(X) _sbrk_r(X)

int _malloc_region_masked(void *r, unsigned int mask);
int default_putc_nobuff(char);

#ifdef INTERNAL_NEWLIB

#include <sys/config.h>
#include <reent.h>

#define RARG struct _reent *reent_ptr,
#define RONEARG struct _reent *reent_ptr
#define RCALL reent_ptr,
#define RONECALL reent_ptr

#define MALLOC_LOCK __malloc_lock(reent_ptr)
#define MALLOC_UNLOCK __malloc_unlock(reent_ptr)

#define RERRNO reent_ptr->_errno

#define nano_malloc		_malloc_r
#define nano_free		_free_r
#define nano_realloc		_realloc_r
#define nano_memalign		_memalign_r
#define nano_valloc		_valloc_r
#define nano_pvalloc		_pvalloc_r
#define nano_calloc		_calloc_r
#define nano_cfree		_cfree_r
#define nano_malloc_usable_size _malloc_usable_size_r
#define nano_malloc_stats	_malloc_stats_r
#define nano_mallinfo		_mallinfo_r
#define nano_mallopt		_mallopt_r

#else /* ! INTERNAL_NEWLIB */

#define RARG
#define RONEARG
#define RCALL
#define RONECALL
#define MALLOC_LOCK
#define MALLOC_UNLOCK
#define RERRNO errno

#define nano_malloc		malloc
#define nano_free		free
#define nano_realloc		realloc
#define nano_memalign		memalign
#define nano_valloc		valloc
#define nano_pvalloc		pvalloc
#define nano_calloc		calloc
#define nano_cfree		cfree
#define nano_malloc_usable_size malloc_usable_size
#define nano_malloc_stats	malloc_stats
#define nano_mallinfo		mallinfo
#define nano_mallopt		mallopt
#endif /* ! INTERNAL_NEWLIB */

/* Redefine names to avoid conflict with user names */
#define free_list __malloc_free_list
#define sbrk_start __malloc_sbrk_start
#define current_mallinfo __malloc_current_mallinfo

// give back an aligned dimension a bit larger than the original;
#define ALIGN_TO(size, align) \
    (((size) + (align) -1L) & ~((align) -1L))

/* Alignment of allocated block */
#define MALLOC_ALIGN (8U)
//#define MALLOC_ALIGN (4U) // changed by gra - 27/3. Alignement to 32 bit -> 4 bytes 
#define CHUNK_ALIGN (sizeof(void*)) // 4 bytes
#define MALLOC_PADDING ((MAX(MALLOC_ALIGN, CHUNK_ALIGN)) - CHUNK_ALIGN)  // for 8U: 8-4

/* as well as the minimal allocation size
 * to hold a free pointer */
#define MALLOC_MINSIZE (sizeof(void *))
#define MALLOC_PAGE_ALIGN (0x1000)
#define MAX_ALLOC_SIZE (0x80000000U)

#define NANO_MAGIC_MASK   0xfffc0000
#define NANO_SIZE_MASK    0x0003ffff

#define NANO_MAGIC        0x56a80000


typedef size_t malloc_size_t;

typedef struct malloc_chunk
{
    /*          --------------------------------------
     *   chunk->| size                               |
     *          --------------------------------------
     *          | Padding for alignment              |
     *          | This includes padding inserted by  |
     *          | the compiler (to align fields) and |
     *          | explicit padding inserted by this  |
     *          | implementation. If any explicit    |
     *          | padding is being used then the     |
     *          | sizeof (size) bytes at             |
     *          | mem_ptr - CHUNK_OFFSET must be     |
     *          | initialized with the negative      |
     *          | offset to size.                    |
     *          --------------------------------------
     * mem_ptr->| When allocated: data               |
     *          | When freed: pointer to next free   |
     *          | chunk                              |
     *          --------------------------------------
     */
    /* size of the allocated payload area, including size before
       CHUNK_OFFSET */
    /* In the esp environment size cannot be bigger than 260K... 
     * so we manage the size so that the 14 MSB are a Magic, and the 
     * size is left to the others 18...
     */
    long size;

    /* since here, the memory is either the next free block, or data load */
    struct malloc_chunk * next;
    #ifdef _NANOMALLOC_BEST_FIT
    #endif /* _NANOMALLOC_BEST_FIT */
}chunk;


// offset of the 'next' pointer into the chunk
#define CHUNK_OFFSET ((malloc_size_t)(&(((struct malloc_chunk *)0)->next)))

/* size of smallest possible chunk. A memory piece smaller than this size
 * won't be able to create a chunk */
#define MALLOC_MINCHUNK (CHUNK_OFFSET + MALLOC_PADDING + MALLOC_MINSIZE)

/* Forward data declarations */
extern chunk * free_list;
extern char * sbrk_start;
extern struct mallinfo current_mallinfo;

#ifdef _NANOMALLOC_BEST_FIT
extern unsigned nano_malloc_region_total;
extern unsigned nano_malloc_region_free;
#endif

#ifdef _NANOMALLOC_FIRST_FREE
extern unsigned nano_malloc_region_total_0;
extern unsigned nano_malloc_region_free_0;
extern unsigned nano_malloc_region_total_1;
extern unsigned nano_malloc_region_free_1;
#endif


/* Forward function declarations */
extern void * nano_malloc(RARG malloc_size_t);
extern void nano_free (RARG void * free_p);
extern void nano_cfree(RARG void * ptr);
extern void * nano_calloc(RARG malloc_size_t n, malloc_size_t elem);
extern void nano_malloc_stats(RONEARG);
extern malloc_size_t nano_malloc_usable_size(RARG void * ptr);
extern void * nano_realloc(RARG void * ptr, malloc_size_t size);
extern void * nano_memalign(RARG size_t align, size_t s);
extern int nano_mallopt(RARG int parameter_number, int parameter_value);
extern void * nano_valloc(RARG size_t s);
extern void * nano_pvalloc(RARG size_t s);



#ifdef DEFINE_MALLINFO
// gra - 27/3/2018
// We re-use usmblks(malloc) and fsmblks(free) (not used in this implementation) to count 
// accesses to malloc and free. Differences are memory leaks!
struct mallinfo current_mallinfo={0,0,0,0,0,0,0,0,0,0};
#endif


static inline chunk * get_chunk_from_ptr(void * ptr)
{
    /* Assume that there is no explicit padding in the
       chunk, and that the chunk starts at ptr - CHUNK_OFFSET.  */
    chunk * c = (chunk *)((char *)ptr - CHUNK_OFFSET);

    /* c->size being negative indicates that there is explicit padding in
       the chunk. In which case, c->size is currently the negative offset to
       the true size.  */
    if (c->size < 0) 
      c = (chunk *)((char *)c + c->size);
    return c;
}

#ifdef DEFINE_MALLOC
/* List list header of free blocks */
chunk * free_list = NULL;

/* Starting point of memory allocated from system */
char * sbrk_start = NULL;

uint32_t MallocCall=0;
uint32_t FreeCall=0;

#ifdef _NANOMALLOC_BEST_FIT
unsigned nano_malloc_region_total;
unsigned nano_malloc_region_free;
#endif

#ifdef _NANOMALLOC_FIRST_FREE
 unsigned nano_malloc_region_total_0;
 unsigned nano_malloc_region_free_0;
 unsigned nano_malloc_region_total_1;
 unsigned nano_malloc_region_free_1;
#endif

/** Function sbrk_aligned
  * Algorithm:
  *   Use sbrk() to obtain more memory and ensure it is CHUNK_ALIGN aligned
  *   Optimise for the case that it is already aligned - only ask for extra
  *   padding after we know we need it
  */

static void* sbrk_aligned(RARG malloc_size_t s)
{
    char *p, *align_p;

    if (sbrk_start == NULL) 
      sbrk_start = _SBRK_R(RCALL 0);

    p = _SBRK_R(RCALL s);

    /* sbrk returns -1 if fail to allocate */
    if (p == (void *)-1)
    {

      return p;
    }

    align_p = (char*)ALIGN_TO((unsigned long)p, CHUNK_ALIGN);
    if (align_p != p)
    {
        /* p is not aligned, ask for a few more bytes so that we have s
         * bytes reserved from align_p. */
        p = _SBRK_R(RCALL align_p - p);
        if (p == (void *)-1)
        {

          return p;
        }
    }

    return align_p;
}



#ifdef _NANOMALLOC_BEST_FIT
/** Function nano_malloc
  * Algorithm:
  *   Walk through the free list to find the first match. If fails to find
  *   one, call sbrk to allocate a new chunk.
  */

static char * EndOfHeap=NULL;

void * nano_malloc(RARG malloc_size_t s)
{
    chunk *p, *r, *b, *bp, *n;
    char * ptr, * align_ptr;
    int offset, rem;

    malloc_size_t alloc_size, best_size;
    
    #ifdef DEFINE_MALLINFO
//    current_mallinfo.usmblks++;
    MallocCall++;
    #endif

    // #define CHUNK_ALIGN (sizeof(void*)) // 4 bytes
    alloc_size = ALIGN_TO(s, CHUNK_ALIGN); /* size of aligned data load */
    // #define MALLOC_PADDING ((MAX(MALLOC_ALIGN, CHUNK_ALIGN)) - CHUNK_ALIGN)  // for 8U: 8-4
    alloc_size += MALLOC_PADDING; /* padding */
    // #define CHUNK_OFFSET ((malloc_size_t)(&(((struct malloc_chunk *)0)->next)))
    alloc_size += CHUNK_OFFSET; /* size of chunk head */
    // #define MALLOC_MINCHUNK (CHUNK_OFFSET + MALLOC_PADDING + MALLOC_MINSIZE)
    alloc_size = MAX(alloc_size, MALLOC_MINCHUNK);


    //puthexint_nobuff(s);
    //puts_nobuff("-\n");


    if (alloc_size >= MAX_ALLOC_SIZE || alloc_size < s)
    {
        RERRNO = ENOMEM;
        return NULL;
    }

    MALLOC_LOCK;

    p = free_list;
    r = p;

    best_size=0;
    b=NULL;

    // first of all search the best chunk

    while (r)
    {
        rem = r->size - alloc_size;

        if(rem >= 0 && (b==NULL || r->size < best_size))
        {
          bp=p;
          b=r;
          best_size=r->size;
          if (rem==0)
            break; // best then equal is impossible!
        }

        if(r->next!=NULL)
        {
          p=r;
          r=r->next;
        }
        else 
          break;
    }


    /* Failed to find an appropriate chunk. Ask for more memory */
    if (b == NULL)
    {
      bp=p;
      // get the last chunk p and get enough mem to fit the difference between the requested size
      // and the available size, if the last chunk is really free... 
      // aligned ... really need to be aligned?
      // Check if 'p' point to the last chunk
      // we need to request more memory
      // _sbrk_r (struct _reent *r, ptrdiff_t incr)

      ptrdiff_t incr=0;
      // If is the first assignment, r will be null
      // if _sbrbk_r cannot allocate mem, we have _errno = ENOMEM
      if(r!=NULL && (((char *) r) + r->size)==_sbrk_r(reent_ptr, 0))
      { // Yes, last chunk. we can get just the difference... if next seg start at the same address...
          incr=alloc_size - r->size;
          _sbrk_r (reent_ptr, incr);
          b=r;
      }
      else 
      {
          b=_sbrk_r (reent_ptr, (ptrdiff_t) alloc_size);
          if(r!=NULL && reent_ptr->_errno==0)
          {
            // only if r is != NULL or we have a problem...
            r->next=b;
          }
          bp=r;
      }

      if(reent_ptr->_errno == 0)
      {
        // got it. 
        b->size=alloc_size;
        b->next=NULL;
        EndOfHeap=((char *) b) + alloc_size;
        nano_malloc_region_total += alloc_size;
        nano_malloc_region_free += alloc_size;
      }
      else
      {
        RERRNO = ENOMEM;
        MALLOC_UNLOCK;
        return NULL;
      }

    }

    // now we have in b the best fitted chunk

 

    rem = b->size - alloc_size;

    if (rem >= MALLOC_MINCHUNK)
      {
        /* Find a chunk that much larger than required size, break
         * it into two chunks and return the first one.  
         * By this way, any added chunk can be added merging chunks*/
        // b is the designed chunk. n is the new free chunk
        n = (chunk *)((char *)b + alloc_size);
        n->size = rem;
        n->next = b->next;
        b->next = n;
        b->size = alloc_size;
        // splitted the region in two parts. the first is b, the second is n
      }
    /* Find a chunk that is exactly the size or slightly bigger
     * than requested size, just return this chunk */

    if (bp == b || free_list==NULL) 
      {
        /* Now it implies p==r==free_list. Move the free_list
         * to next chunk */
        free_list = b->next;
      }
    else
      {
        /* Normal case. Remove it from free_list, except if it's the first chunk... */
        if(bp!=NULL)
          bp->next = b->next;
      }


      /* Dram */
    nano_malloc_region_free -= b->size;


    MALLOC_UNLOCK;

    ptr = (char *)b + CHUNK_OFFSET; // poiter to the data section... not aligned
    /*
    // get the first size/address aligned with the 'align'
    #define ALIGN_TO(size, align) \
    (((size) + (align) -1L) & ~((align) -1L))
    */

    align_ptr = (char *)ALIGN_TO((unsigned long)ptr, MALLOC_ALIGN); // aligned pointer to the data section... in the middle, the padding
    // Aligning the pointer, 'moves' the pointer forward enough to comply with the alignement request.
    // Can actually be just '4', as value...
    offset = align_ptr - ptr;
#ifdef _LIBC_DEBUG_HEAP

    puts_nobuff("ptr: ");
    puthexint_nobuff(b);
    puts_nobuff("; alg: ");
    puthexint_nobuff(align_ptr);
    puts_nobuff("; size: ");
    puthexint_nobuff(s);

//    puts_nobuff("; Caller: ");
//    puthexint_nobuff(Caller);
    puts_nobuff("\n");
#endif /* #ifdef _LIBC_DEBUG_HEAP */

    if (offset)
    {
 
        /* Initialize sizeof (malloc_chunk.size) bytes at
           align_ptr - CHUNK_OFFSET with negative offset to the
           size field (at the start of the chunk).

           The negative offset to size from align_ptr - CHUNK_OFFSET is
           the size of any remaining padding minus CHUNK_OFFSET.  This is
           equivalent to the total size of the padding, because the size of
           any remaining padding is the total size of the padding minus
           CHUNK_OFFSET.

           Note that the size of the padding must be at least CHUNK_OFFSET.

           The rest of the padding is not initialized.  */
        *(long *)((char *)b + offset) = -offset;
    }

    assert(align_ptr + s <= (char *)ptr + alloc_size);
//    assert(!(align_ptr & 0x07)); // not aligned to MALLOC_ALIGN...

//    iprintf("*************> Malloc. Assigned %p, size %d, offset: %d. Allocated: %d. Chunk Addr: %p\n", align_ptr, s, offset, alloc_size, ptr);

    return align_ptr;
}
#endif /* _NANOMALLOC_BEST_FIT */



#ifdef _NANOMALLOC_FIRST_FREE
/** Function nano_malloc
  * Algorithm:
  *   Walk through the free list to find the first match. If fails to find
  *   one, call sbrk to allocate a new chunk.
  */
void * nano_malloc(RARG malloc_size_t s)
{
    chunk *p, *r;
    char * ptr, * align_ptr;
    int offset;

    malloc_size_t alloc_size;

    unsigned int mask, pre_mask, post_mask;
    


    #ifdef DEFINE_MALLINFO
    current_mallinfo.usmblks++;
    #endif



    alloc_size = ALIGN_TO(s, CHUNK_ALIGN); /* size of aligned data load */
    alloc_size += MALLOC_PADDING; /* padding */
    alloc_size += CHUNK_OFFSET; /* size of chunk head */
    alloc_size = MAX(alloc_size, MALLOC_MINCHUNK);

    if (alloc_size >= MAX_ALLOC_SIZE || alloc_size < s)
    {
        RERRNO = ENOMEM;
        return NULL;
    }

    MALLOC_LOCK;

    p = free_list;
    r = p;


    /* Two mask sets are packed into one. If these are equal then only
     * one pass is made searching for a free region. This allows a
     * preferred set to be search first. The pre_mask set is a subset
     * of the post_mask set - this is enforced. */
    mask = reent_ptr->malloc_region_mask;
    pre_mask = mask >> 16;
    post_mask = mask & pre_mask;



    if (nano_malloc_region_free_0 < 12 * 1024 && (post_mask & 2) != 2) {
      /* Search elsewhere first. */
      pre_mask = 0xfffd;
    }

    /* If the pre pass would check the same set or nothing then skip it. */
    if (pre_mask != post_mask && pre_mask != 0xffff) 
    {

     while (r)
	   {

	     if (_malloc_region_masked(r, pre_mask))
            {
              p=r;
              r=r->next;
              continue;
            }


          int rem = r->size - alloc_size;
          if (rem >= 0)
            {
              if (rem >= MALLOC_MINCHUNK)
                {
                  /* Find a chunk that much larger than required size, break
                   * it into two chunks and return the second one */
                  r->size = rem;
                  r = (chunk *)((char *)r + rem);
                  r->size = alloc_size;
                }
              /* Find a chunk that is exactly the size or slightly bigger
               * than requested size, just return this chunk */
              else if (p == r)
                {
                  /* Now it implies p==r==free_list. Move the free_list
                   * to next chunk */
                  free_list = r->next;
                }
              else
                {
                  /* Normal case. Remove it from free_list */
                  p->next = r->next;
                }
              break;
            }
          p=r;
          r=r->next;
    }
  } else {
      r = NULL;
  }

  if (r == NULL) {
      /* try again. */
      p = free_list;
      r = p;

      while (r)
        {

          if (mask && _malloc_region_masked(r, post_mask))
            {
              p = r;
              r = r->next;
              continue;
            }


          int rem = r->size - alloc_size;
          if (rem >= 0)
            {
              if (rem >= MALLOC_MINCHUNK)
                {
                  /* Find a chunk that much larger than required size, break
                   * it into two chunks and return the second one */
                  r->size = rem;
                  r = (chunk *)((char *)r + rem);
                  r->size = alloc_size;
                }
              /* Find a chunk that is exactly the size or slightly bigger
               * than requested size, just return this chunk */
              else if (p == r)
                {
                  /* Now it implies p==r==free_list. Move the free_list
                   * to next chunk */
                  free_list = r->next;
                }
              else
                {
                  /* Normal case. Remove it from free_list */
                  p->next = r->next;
                }
              break;
            }
          p = r;
          r = r->next;
        }
  }

    /* Failed to find an appropriate chunk. Ask for more memory */
    if (r == NULL)
    {
    	RERRNO = ENOMEM;
    	MALLOC_UNLOCK;
    	return NULL;
    }

    /* Account for the allocation. TODO avoid baking in constants here?? */
    if ((uintptr_t)r < 0x40000000) {
      /* Dram */
      nano_malloc_region_free_0 -= r->size;
    } else {
      nano_malloc_region_free_1 -= r->size;
    }

    MALLOC_UNLOCK;

    ptr = (char *)r + CHUNK_OFFSET;

    align_ptr = (char *)ALIGN_TO((unsigned long)ptr, MALLOC_ALIGN);
    offset = align_ptr - ptr;

    if (offset)
    {
        /* Initialize sizeof (malloc_chunk.size) bytes at
           align_ptr - CHUNK_OFFSET with negative offset to the
           size field (at the start of the chunk).

           The negative offset to size from align_ptr - CHUNK_OFFSET is
           the size of any remaining padding minus CHUNK_OFFSET.  This is
           equivalent to the total size of the padding, because the size of
           any remaining padding is the total size of the padding minus
           CHUNK_OFFSET.

           Note that the size of the padding must be at least CHUNK_OFFSET.

           The rest of the padding is not initialized.  */
        *(long *)((char *)r + offset) = -offset;
    }

    assert(align_ptr + size <= (char *)r + alloc_size);

    return align_ptr;
}
#endif

#endif /* DEFINE_MALLOC */

#ifdef DEFINE_FREE
/*
extern unsigned nano_malloc_region_total_0;
extern unsigned nano_malloc_region_free_0;

#ifdef _NANOMALLOC_FIRST_FREE
extern unsigned nano_malloc_region_total_1;
extern unsigned nano_malloc_region_free_1;
#endif
*/
#define MALLOC_CHECK_WRONG_FREE

void puts_nobuff(char *s); 
/** Function nano_free
  * Implementation of libc free.
  * Algorithm:
  *  Maintain a global free chunk single link list, headed by global
  *  variable free_list.
  *  When free, insert the to-be-freed chunk into free list. The place to
  *  insert should make sure all chunks are sorted by address from low to
  *  high.  Then merge with neighbor chunks if adjacent.
  *
  * ToDo: Check if is a pointer before free it
  */
void nano_free (RARG void * free_p)
{
    chunk * p_to_free;
    chunk * p, * q;
    int size_freed;


    if (free_p == NULL) return;

    #ifdef DEFINE_MALLINFO
    FreeCall++;
    #endif


    p_to_free = get_chunk_from_ptr(free_p);

#ifdef _LIBC_DEBUG_HEAP

    iprintf("*************> Freeing %p. From %p, size %d\n", p_to_free, free_p, p_to_free->size);

#endif /* #ifdef _LIBC_DEBUG_HEAP */

    if(((uint32_t) p_to_free & 0x03)  || ((uint32_t) free_p & 0x07))
    {
      iprintf("*************> UNALIGNED POINTER! Waiting ***************\n");
      abort();
      while(1);
    }

    MALLOC_LOCK;

#ifdef _NANOMALLOC_FIRST_FREE
    /* Account for the allocation */
    if ((uintptr_t)free_p < 0x40000000) {
      /* Dram */
      nano_malloc_region_free_0 += p_to_free->size;
    } else {
      nano_malloc_region_free_1 += p_to_free->size;
    }
#endif


#ifdef _NANOMALLOC_BEST_FIT
//    nano_malloc_region_free += p_to_free->size;
#endif /* _NANOMALLOC_BEST_FIT */

    size_freed=p_to_free->size;

    if (free_list == NULL)
    {
        /* Set first free list element */
        p_to_free->next = free_list;
        free_list = p_to_free;
        nano_malloc_region_free += size_freed;
        MALLOC_UNLOCK;
        return;
    }

    if (p_to_free < free_list)
    {
        if ((char *)p_to_free + p_to_free->size == (char *)free_list)
        {
            /* Chunk to free is just before the first element of
             * free list  */
            p_to_free->size += free_list->size;
            p_to_free->next = free_list->next;
        }
        else
        {
            /* Insert before current free_list */
            p_to_free->next = free_list;
        }
        nano_malloc_region_free += size_freed;
        free_list = p_to_free;
        MALLOC_UNLOCK;
        return;
    }

    q = free_list;
    /* Walk through the free list to find the place for insert. */
    do
    {
        p = q;
        q = q->next;
    } while (q && q <= p_to_free);

    /* Now p <= p_to_free and either q == NULL or q > p_to_free
     * Try to merge with chunks immediately before/after it. */

    if ((char *)p + p->size == (char *)p_to_free)
    {
        /* Chunk to be freed is adjacent
         * to a free chunk before it */
        p->size += p_to_free->size;
        /* If the merged chunk is also adjacent
         * to the chunk after it, merge again */
        if ((char *)p + p->size == (char *) q)
        {
            p->size += q->size;
            p->next = q->next;
        }
    }
#ifdef MALLOC_CHECK_WRONG_FREE
    else if ((char *)p + p->size > (char *)p_to_free)
    {
        /* Report double free fault */
        iprintf("Warning: freeing %p - size 0x%x error\n", p, p->size);
        RERRNO = ENOMEM;
        MALLOC_UNLOCK;
        return;
    }
#endif
    else if ((char *)p_to_free + p_to_free->size == (char *) q)
    {
        /* Chunk to be freed is adjacent
         * to a free chunk after it */
        p_to_free->size += q->size;
        p_to_free->next = q->next;
        p->next = p_to_free;
    }
    else
    {
        /* Not adjacent to any chunk. Just insert it. Resulting
         * a fragment. */
        p_to_free->next = q;
        p->next = p_to_free;
    }
    nano_malloc_region_free += size_freed;
    MALLOC_UNLOCK;
    return;
}

#ifdef _NANOMALLOC_BEST_FIT
/* Insert a disjoint region into the nano malloc pool. Create a malloc chunk,
 * filling the size as newlib nano malloc expects, and then free it. */
void nano_malloc_insert_chunk(void *start, size_t size)
{
    /* Align the chunk, and adjust the size. */


    chunk *aligned_start = (void *)ALIGN_TO((uintptr_t)start, CHUNK_ALIGN);
    chunk *p;
    size_t offset = (uintptr_t)aligned_start - (uintptr_t)start;
    size_t aligned_size = size - offset;


    if (aligned_size < MALLOC_MINCHUNK) 
      return;   

    aligned_start->size=aligned_size;
 
    if (start <= free_list || free_list==NULL)
    {
      aligned_start->next=free_list;
      free_list=aligned_start;
    }
    else
    {
      // search the last allocated region
      p=free_list;
      while(p->next!=NULL) // search the last chunk
        p=p->next;
      // attach the new chunk to heap
      p->next=aligned_start;
      aligned_start->next=NULL;

    }

    nano_malloc_region_total += aligned_size;
    nano_malloc_region_free += aligned_size;
}
#endif /* _NANOMALLOC_BEST_FIT */


#ifdef _NANOMALLOC_FIRST_FREE
/* Insert a disjoint region into the nano malloc pool. Create a malloc chunk,
 * filling the size as newlib nano malloc expects, and then free it. */
void nano_malloc_insert_chunk(void *start, size_t size)
{
    /* Align the chunk, and adjust the size. */
    chunk *aligned_start = (void *)ALIGN_TO((uintptr_t)start, CHUNK_ALIGN);
    size_t offset = (uintptr_t)start - (uintptr_t)aligned_start;
    size_t aligned_size = size - offset;

    if (aligned_size < MALLOC_MINCHUNK) return;

    aligned_start->size = aligned_size;
    free((uint8_t *)aligned_start + CHUNK_OFFSET);

    /* Account for the allocation. */
    /* TODO abstract this; avoid baking in port specific constants. */
    if ((uintptr_t)aligned_start < 0x40000000)
    {
        /* Dram */
        nano_malloc_region_total_0 += aligned_size;
    } else {
        nano_malloc_region_total_1 += aligned_size;
    }
}
#endif

#endif /* DEFINE_FREE */

#ifdef DEFINE_CFREE
void nano_cfree(RARG void * ptr)
{
    nano_free(RCALL ptr);
}
#endif /* DEFINE_CFREE */

#ifdef DEFINE_CALLOC
/* Function nano_calloc
 * Implement calloc simply by calling malloc and set zero */
void * nano_calloc(RARG malloc_size_t n, malloc_size_t elem)
{
    void * mem = nano_malloc(RCALL n * elem);
    if (mem != NULL) memset(mem, 0, n * elem);
    return mem;
}
#endif /* DEFINE_CALLOC */

#ifdef DEFINE_REALLOC
/* Function nano_realloc
 * Implement realloc by malloc + memcpy */
void * nano_realloc(RARG void * ptr, malloc_size_t size)
{
    void * mem;
    chunk * p_to_realloc;

    if (ptr == NULL) return nano_malloc(RCALL size);

    if (size == 0)
    {
        nano_free(RCALL ptr);
        return NULL;
    }

    /* TODO: There is chance to shrink the chunk if newly requested
     * size is much small */
    if (nano_malloc_usable_size(RCALL ptr) >= size)
      return ptr;

    mem = nano_malloc(RCALL size);
    if (mem != NULL)
    {
        memcpy(mem, ptr, size);
        nano_free(RCALL ptr);
    }
    return mem;
}
#endif /* DEFINE_REALLOC */

#ifdef DEFINE_MALLINFO


struct mallinfo nano_mallinfo(RONEARG)
{
    char * sbrk_now;
    chunk * pf;
    size_t free_size = 0;
    size_t total_size = 0; // initialized total_size - gra

#ifdef _NANOMALLOC_BEST_FIT
    unsigned int mask, pre_mask, post_mask;

    mask = reent_ptr->malloc_region_mask;
    pre_mask = mask >> 16;
    post_mask = mask & pre_mask;
#endif /* _NANOMALLOC_BEST_FIT */


    MALLOC_LOCK;

#ifdef _NANOMALLOC_FIRST_FREE
    if ((post_mask & 1) == 0) {
      total_size += nano_malloc_region_total_0;
      free_size += nano_malloc_region_free_0;
    }

    if ((post_mask & 2) == 0) {
      total_size += nano_malloc_region_total_1;
      free_size += nano_malloc_region_free_1;
    }
#endif /* _NANOMALLOC_FIRST_FREE */ 



#ifdef _NANOMALLOC_BEST_FIT
      total_size = nano_malloc_region_total;
      free_size = nano_malloc_region_free;
#endif /* _NANOMALLOC_BEST_FIT */

    current_mallinfo.arena = total_size;
    current_mallinfo.fordblks = free_size;
    current_mallinfo.uordblks = total_size - free_size;
 //   current_mallinfo.usmblks = MallocCall;
 //   current_mallinfo.fsmblks = FreeCall;


    MALLOC_UNLOCK;
    return current_mallinfo;
}
#endif /* DEFINE_MALLINFO */

#ifdef DEFINE_MALLOC_STATS
void nano_malloc_stats(RONEARG)
{
    nano_mallinfo(RONECALL);
    fiprintf(stderr, "max system bytes = %10u\n",
             current_mallinfo.arena);
    fiprintf(stderr, "system bytes     = %10u\n",
             current_mallinfo.arena);
    fiprintf(stderr, "in use bytes     = %10u\n",
             current_mallinfo.uordblks);
}
#endif /* DEFINE_MALLOC_STATS */

#ifdef DEFINE_MALLOC_USABLE_SIZE
malloc_size_t nano_malloc_usable_size(RARG void * ptr)
{
    chunk * c = (chunk *)((char *)ptr - CHUNK_OFFSET);
    int size_or_offset = c->size;

    if (size_or_offset < 0)
    {
        /* Padding is used. Excluding the padding size */
        c = (chunk *)((char *)c + c->size);
        return c->size - CHUNK_OFFSET + size_or_offset;
    }
    return c->size - CHUNK_OFFSET;
}
#endif /* DEFINE_MALLOC_USABLE_SIZE */

#ifdef DEFINE_MEMALIGN
/* Function nano_memalign
 * Allocate memory block aligned at specific boundary.
 *   align: required alignment. Must be power of 2. Return NULL
 *          if not power of 2. Undefined behavior is bigger than
 *          pointer value range.
 *   s: required size.
 * Return: allocated memory pointer aligned to align
 * Algorithm: Malloc a big enough block, padding pointer to aligned
 *            address, then truncate and free the tail if too big.
 *            Record the offset of align pointer and original pointer
 *            in the padding area.
 */
void * nano_memalign(RARG size_t align, size_t s)
{
    chunk * chunk_p;
    malloc_size_t size_allocated, offset, ma_size, size_with_padding;
    char * allocated, * aligned_p;

    /* Return NULL if align isn't power of 2 */
    if ((align & (align-1)) != 0) return NULL;

    align = MAX(align, MALLOC_ALIGN);
    ma_size = ALIGN_TO(MAX(s, MALLOC_MINSIZE), CHUNK_ALIGN);
    size_with_padding = ma_size + align - MALLOC_ALIGN;

    allocated = nano_malloc(RCALL size_with_padding);
    if (allocated == NULL) return NULL;

    chunk_p = get_chunk_from_ptr(allocated);
    aligned_p = (char *)ALIGN_TO(
                  (unsigned long)((char *)chunk_p + CHUNK_OFFSET),
                  (unsigned long)align);
    offset = aligned_p - ((char *)chunk_p + CHUNK_OFFSET);

    if (offset)
    {
        if (offset >= MALLOC_MINCHUNK)
        {
            /* Padding is too large, free it */
            chunk * front_chunk = chunk_p;
            chunk_p = (chunk *)((char *)chunk_p + offset);
            chunk_p->size = front_chunk->size - offset;
            front_chunk->size = offset;
            nano_free(RCALL (char *)front_chunk + CHUNK_OFFSET);
        }
        else
        {
            /* Padding is used. Need to set a jump offset for aligned pointer
            * to get back to chunk head */
            assert(offset >= sizeof(int));
            *(long *)((char *)chunk_p + offset) = -offset;
        }
    }

    size_allocated = chunk_p->size;
    if ((char *)chunk_p + size_allocated >
         (aligned_p + ma_size + MALLOC_MINCHUNK))
    {
        /* allocated much more than what's required for padding, free
         * tail part */
        chunk * tail_chunk = (chunk *)(aligned_p + ma_size);
        chunk_p->size = aligned_p + ma_size - (char *)chunk_p;
        tail_chunk->size = size_allocated - chunk_p->size;
        nano_free(RCALL (char *)tail_chunk + CHUNK_OFFSET);
    }
    return aligned_p;
}
#endif /* DEFINE_MEMALIGN */

#ifdef DEFINE_MALLOPT
int nano_mallopt(RARG int parameter_number, int parameter_value)
{
    return 0;
}
#endif /* DEFINE_MALLOPT */

#ifdef DEFINE_VALLOC
void * nano_valloc(RARG size_t s)
{
    return nano_memalign(RCALL MALLOC_PAGE_ALIGN, s);
}
#endif /* DEFINE_VALLOC */

#ifdef DEFINE_PVALLOC
void * nano_pvalloc(RARG size_t s)
{
    return nano_valloc(RCALL ALIGN_TO(s, MALLOC_PAGE_ALIGN));
}
#endif /* DEFINE_PVALLOC */
