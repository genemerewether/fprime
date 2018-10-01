extern "C" {
  void sw_main(void);
}

// Here to get Unix builds to complete without errors

int main(int argc, char* argv[]) {
  sw_main();
  return 0;
}
