//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef ENSURE_BUFFER_SIZE_H
#define ENSURE_BUFFER_SIZE_H

//#include <vector>

//-----------------------------------------------------------


template <typename T, typename Allocator = std::allocator<T>>
void ensure_buffer_size(size_t sizeEnsure, std::vector<T, Allocator>& buffer)
{
	if( buffer.capacity() < sizeEnsure ) buffer.reserve(2*sizeEnsure);
	if( buffer.size() < sizeEnsure ) buffer.resize(sizeEnsure);

}

//-----------------------------------------------------------


#endif // ENSURE_BUFFER_SIZE_H
