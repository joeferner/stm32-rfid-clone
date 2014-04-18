
#include "CuTest.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern CuSuite* em4x05_suite();

int main(int argc, const char* args[]) {
  CuString *output = CuStringNew();
  CuSuite* suite = CuSuiteNew();

  CuSuiteAddSuite(suite, em4x05_suite());

  CuSuiteRun(suite);
  CuSuiteSummary(suite, output);
  CuSuiteDetails(suite, output);
  printf("%s\n", output->buffer);
}
