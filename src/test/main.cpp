#include "gtest/gtest.h"
#include "lion/thirdparty/include/logger.hpp"

bool is_valgrind = false;

int main(int argc, char **argv) {

  for (int i = 0; i < argc; ++i)
  {
     if ( !strcmp(argv[i], "--valgrind") )
     {
        is_valgrind = true;
     }
  }

  out.set_print_level(2);
    
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
