extern "C" {
  void fsw_main(void);
}

// Here to get Unix builds to complete without errors

int main(int argc, char* argv[]) {
  fsw_main();
  return 0;
}
