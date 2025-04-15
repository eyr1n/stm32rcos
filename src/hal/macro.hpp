#pragma once

#define DECLARE_CONTEXT(handle, instance)                                      \
  if ((handle)->Instance == (instance)) {                                      \
    static void *context;                                                      \
    return &context;                                                           \
  }