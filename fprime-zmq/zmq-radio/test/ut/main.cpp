
#include <fprime-zmq/zmq-radio/test/ut/Tester.hpp>

TEST(zmqradio,testConnection) {
    Zmq::Tester test;
    test.testConnection();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


