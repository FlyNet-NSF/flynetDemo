#include "Image.h"
#include "Exceptions.h"

using namespace npp;

    bool
    operator== (const Image::Size &rFirst, const Image::Size &rSecond)
    {
        return rFirst.nWidth == rSecond.nWidth && rFirst.nHeight == rSecond.nHeight;
    }

    bool
    operator!= (const Image::Size &rFirst, const Image::Size &rSecond)
    {
        return rFirst.nWidth != rSecond.nWidth || rFirst.nHeight != rSecond.nHeight;
    }



    /// Output stream inserter for Exception.
    /// \param rOutputStream The stream the exception information is written to.
    /// \param rException The exception that's being written.
    /// \return Reference to the output stream being used.
    std::ostream &
    operator << (std::ostream &rOutputStream, const npp::Exception &rException)
    {
        rOutputStream << rException.toString();
        return rOutputStream;
    }

