#pragma once
#include "common.cuh"

NORI_NAMESPACE_BEGIN

void *AllocAligned(size_t size);
template <typename T>
T *AllocAligned(size_t count){
    return (T*)AllocAligned(count * sizeof(T));
}
void FreeAligned(void *);
class MemoryArena {
    public:
        MemoryArena(size_t blockSize = 262144) : blockSize(blockSize) {}
        ~MemoryArena(){
            FreeAligned(currentBlock);
            for(auto &block : usedBlocks) FreeAligned(block.second);
            for(auto &block : availableBlocks) FreeAligned(block.second);
        }

        void *Alloc(size_t nBytes){
        #if __GNUC__ == 4 && __GNUC_MINOR__ < 9
        // gcc bug: max_align_t wasn't in std:: until 4.9.0
            const int align = alignof(::max_align_t);
        #else
            const int align = alignof(std::max_align_t);
        #endif
            nBytes = (nBytes + align - 1) & ~(align - 1);
            if(currentBlockPos + nBytes > currentAllocSize){
                if(currentBlock){
                    usedBlocks.push_back(std::make_pair(currentAllocSize, currentBlock));
                    currentBlock = nullptr;
                    currentAllocSize = 0;
                }

                for(auto iter = availableBlocks.begin(); iter != availableBlocks.end(); iter++){
                    if(nBytes <= iter->first){
                        currentAllocSize = iter->first;
                        currentBlock = iter->second;
                        availableBlocks.erase(iter);
                        break;
                    }
                }
                
                if(!currentBlock){
                    currentAllocSize = std::max(nBytes, blockSize);
                    currentBlock = AllocAligned<uint8_t>(currentAllocSize);
                }
                currentBlockPos = 0;
            }
            void* ret = currentBlock + currentBlockPos;
            currentBlockPos += nBytes;
            return ret;
        }

        template <typename T>
        T *Alloc(size_t n = 1, bool runConstructor = true){
            T *ret = (T *)Alloc(n * sizeof(T));
            if(runConstructor)
                for (size_t i = 0; i < n; i++) new (&ret[i]) T();
            return ret;
        }

        void Reset() {
            currentBlockPos = 0;
            availableBlocks.splice(availableBlocks.begin(), usedBlocks);
        }

        size_t TotalAllocated() const {
            size_t total = currentAllocSize;
            for (const auto &alloc : usedBlocks) total += alloc.first;
            for (const auto &alloc : availableBlocks) total += alloc.first;
            return total;
        }


    private:
        MemoryArena(const MemoryArena &) = delete;
        MemoryArena &operator=(const MemoryArena &) = delete;
        const size_t blockSize;
        size_t currentBlockPos = 0, currentAllocSize = 0;
        uint8_t *currentBlock = nullptr;
        std::list<std::pair<size_t, uint8_t *>> usedBlocks, availableBlocks;
};
NORI_NAMESPACE_END