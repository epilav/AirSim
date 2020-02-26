#ifndef msr_airlib_ArduCopterPacketBuffer_hpp
#define msr_airlib_ArduCopterPacketBuffer_hpp

#include <cassert>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <type_traits>

namespace msr { namespace airlib {

template <std::size_t N>
class ArduCopterPacketBuffer final 
{
public:
    ArduCopterPacketBuffer() : pos_(bytes_), end_(bytes_) {}

    std::size_t getIndex() const
    {
        return pos_ - bytes_;
    }

    void setIndex(std::size_t pos) 
    {
        pos_ = bytes_ + pos;
    }

    const void* getBytes()
    {
        return bytes_;
    }

    std::size_t getSize()
    {
        return end_ - bytes_;
    }

    // Stores the given value at the current position in the buffer. The value is cast to T before it is 
    // written into the buffer. The current position pointer is bumped ahead by sizeof(T) bytes.
    template <
        typename T, 
        typename U, 
        typename std::enable_if<std::is_integral<U>::value>::type* = nullptr>
    void put(U value) 
    {
        assert(pos_ + sizeof(T) < bytes_ + N);
        // TODO(epilav): do byte swapping, if necessary.
        *(reinterpret_cast<T *>(pos_)) = static_cast<T>(value);
        incrementPos(sizeof(T));
    }

    // Stores the given value at the current position in the buffer. The value is cast to T before it is 
    // written into the buffer. The current position pointer is bumped ahead by sizeof(T) bytes.
    template <
        typename T,
        typename U,
        typename std::enable_if<std::is_floating_point<U>::value>::type* = nullptr>
    void put(U value)
    {
        assert(pos_ + sizeof(T) < bytes_ + N);
        *(reinterpret_cast<T *>(pos_)) = static_cast<T>(value);
        incrementPos(sizeof(T));
    }

    // Stores a series of values into the bufffer starting at the current position. All values are cast to 
    // T before being written into the buffer. The current position is incremented by N * sizeof(T) where N
    // is the total number of values written.
    template <typename T, typename HeadArg, typename... TailArgs>
    void put(HeadArg&& head_arg, TailArgs&&... tail_args) 
    {
        put<T>(head_arg);
        put<T>(std::forward<TailArgs>(tail_args)...);
    }

    // Serializes the given string into the buffer. The length of the string, represented by a 16-bit value
    // is written first, followed by the contents of the string. The null character is not copied into
    // the buffer.
    void putString(const std::string& value)
    {
        auto size = value.size();
        put<uint16_t>(size);
        assert(pos_ + size < bytes_ + N);
        std::memcpy(pos_, value.data(), size);
        incrementPos(size);
    }

    // Serializes the given vector into the buffer. The length of the vector, represented by a 32-bit value
    // is written first, followed by the contents of the vectgor.
    template <typename T>
    void putVector(const std::vector<T>& elements)
    {
        putArray<T>(elements.data(), elements.size());
    }

    // Serializes the given array into the buffer. The number of elements, represented by a 32-bit value is
    // written first, followed by the contents of the array. The curren position is incremented by
    // n_elems * sizeof(T) bytes.
    template <typename T>
    void putArray(const T* elements, std::size_t n_elems)
    {
        put<uint32_t>(n_elems);
        assert(pos_ + n_elems * sizeof(T) < bytes_ + N);
        std::memcpy(pos_, elements, n_elems);
        incrementPos(sizeof(T) * n_elems);
    }

    template <typename T>
    T get()
    {
        assert(pos_ + sizeof(T) < bytes_ + N);
        T r = *(reinterpret_cast<T*>(pos_));
        pos_ += sizeof(T);
        return r;
    }

    std::string getString()
    {
        auto size = get<uint32_t>();
        assert(pos_ + size < bytes_ + N);
        return std::string(pos_, size);
    }

    template <typename T>
    std::vector<T> getVector()
    {
        auto size = get<uint32_t>();
        T* start = reinterpret_cast<T *>(pos_);
        pos_ += sizeof(T) * size;
        return std::vector<T>(start, pos_);
    }

protected:
    void incrementPos(std::size_t n)
    {
        pos_ += n;
        end_ = std::max(end_, pos_);
    }

private:
    uint8_t *pos_;
    uint8_t *end_;
    uint8_t bytes_[N];
};

}} // namespace


#endif // msr_airlib_ArduCopterPacketBuffer_hpp