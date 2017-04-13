//
// Utility functions
//
// David Butterworth
//

#ifndef UTILS_IMPL_HPP
#define UTILS_IMPL_HPP

#include <vector>
#include <iostream> // cout

#include <algorithm> // min, max, generate, reverse

#include <boost/lexical_cast.hpp>

template <typename Type>
const std::vector<Type> range(const Type start, const Type end)
{
  const Type min_value =  std::min(start, end);
  const Type max_value =  std::max(start, end);

  const Type num_values = (max_value - min_value) + 1;

  struct Generator
  {
    Type current_;
    Generator (const Type start): current_(start) {}
    Type operator() () { return current_++; }
  };

  // Fill a vector with increasing values
  std::vector<Type> vec(num_values);
  Generator generator(min_value);
  std::generate(vec.begin(), vec.end(), generator);

  if (start > end)
  {
    std::reverse(vec.begin(), vec.end());
    //return vec;
  }

  return vec;
}

template <typename Type>
void printStdVector(const typename std::vector<Type> v)
{
  std::cout << "[";
  for (size_t i = 0; i < v.size(); ++i)
  {
    std::cout << v.at(i);
    if (i != (v.size() - 1))
    {
      std::cout << ", ";
    }
  }
  std::cout << "]" << std::endl;
}

template <typename Type>
void initializeVector(std::vector<std::vector<std::vector<Type> > >& vector,
                      const size_t dim_1_size, const size_t dim_2_size, const size_t dim_3_size,
                      const Type value)
{
  vector.resize(dim_1_size);

  for (size_t i = 0; i < dim_1_size; ++i)
  {
    vector.at(i).resize(dim_2_size);

    for (size_t j = 0; j < dim_2_size; ++j)
    {
      vector.at(i).at(j).resize(dim_3_size);

      for (size_t k = 0; k < dim_3_size; ++k)
      {
        vector.at(i).at(j).at(k) = value;
      }
    }
  }
}

template <typename Type>
void initializeVector(std::vector<std::vector<std::vector<std::vector<Type> > > >& vector,
                      const size_t dim_1_size, const size_t dim_2_size, const size_t dim_3_size, const size_t dim_4_size,
                      const Type value)
{
  vector.resize(dim_1_size);

  for (size_t i = 0; i < dim_1_size; ++i)
  {
    vector.at(i).resize(dim_2_size);

    for (size_t j = 0; j < dim_2_size; ++j)
    {
      vector.at(i).at(j).resize(dim_3_size);

      for (size_t k = 0; k < dim_3_size; ++k)
      {
        //vector.at(i).at(j).at(k) = value;

        vector.at(i).at(j).at(k).resize(dim_4_size);

        for (size_t l = 0; l < dim_4_size; ++l)
        {
          vector.at(i).at(j).at(k).at(l) = value;
        }

      }
    }
  }
}

template <typename Type>
void printVector(std::vector<std::vector<std::vector<Type> > >& vector)
{
  for (size_t i = 0; i < vector.size(); ++i)
  {
    for (size_t j = 0; j < vector.at(0).size(); ++j)
    {
      for (size_t k = 0; k < vector.at(0).at(0).size(); ++k)
      {
        std::cout << " i,j,k: " << i << "," << j << "," << k
                  << " = " << vector.at(i).at(j).at(k)
                  << std::endl;
      }
    }
  }
}

template <typename Type>
void printVector(std::vector<std::vector<std::vector<std::vector<Type> > > >& vector)
{
  for (size_t i = 0; i < vector.size(); ++i)
  {
    for (size_t j = 0; j < vector.at(0).size(); ++j)
    {
      for (size_t k = 0; k < vector.at(0).at(0).size(); ++k)
      {
        for (size_t l = 0; l < vector.at(0).at(0).at(0).size(); ++l)
        {

          std::cout << " i,j,k,l: " << i << "," << j << "," << k << "," << l
                    << " = " << vector.at(i).at(j).at(k).at(l)
                    << std::endl;
        }
      }
    }
  }
}

template <typename Type>
const std::vector<Type> splitStringAsType(const std::string& input, const char delimiter)
{
  std::vector<Type> tokens;

  const std::vector<std::string> substrings = splitString(input, delimiter);
  for (size_t i = 0; i < substrings.size(); ++i)
  {
    tokens.push_back(boost::lexical_cast<Type>(substrings.at(i)));
  }

  return tokens;
}


#endif // UTILS_IMPL_HPP
