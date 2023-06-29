# FBOW

FBOW (Fast Bag of Words) is an extremely optimized version of the DBow2/DBow3 libraries. The library is highly optimized to speed up the Bag of Words creation using AVX, SSE and MMX instructions. In loading a vocabulary, fbow is ~80x faster than DBoW2 (see tests directory and try). In transforming an image into a bag of words using on machines with AVX instructions, it is ~6.4x faster.

## Main features

* Only depends on OpenCV.
* Any type of descriptors allowed out of the box (binary and real).
* Dictionary creation from a set of images; bugs found in DBoW2/3 corrected.
* Extremmely fast bow creation using specialized versions using AVX, SSE and MMX instructions both for binary and floating point descriptors.
* Very fast loading of vocabularies.

## Limitations

* Indexing of images not implemented yet.

## Citing

If you use this project in academic research you must cite us. This project is part of the ucoslam project. Visit [ucoslam.com](http://ucoslam.com) for more information.

## Test speed

Run the program `test_dbow2VSfbow`.

## License

This software is distributed under MIT license. See `LICENSE` file for more information.
