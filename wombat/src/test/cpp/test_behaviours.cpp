// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include <chrono>
#include <thread>

#include "behaviour/Behaviour.h"
#include "behaviour/BehaviourScheduler.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace behaviour;

class MockSystem : public HasBehaviour {};
class MockBehaviour : public Behaviour {
 public:
  MOCK_METHOD0(OnStart, void());
  MOCK_METHOD0_T(OnStop, void());
  MOCK_METHOD1(OnTick, void(units::time::second_t));
};

TEST(BehaviourTest, Tick) {
  auto b = make<MockBehaviour>();

  {
    ::testing::InSequence s;
    EXPECT_CALL(*b, OnStart).Times(1);
    EXPECT_CALL(*b, OnTick).Times(4);
    EXPECT_CALL(*b, OnStop).Times(1);
  }

  EXPECT_FALSE(b->Tick());
  EXPECT_FALSE(b->Tick());
  EXPECT_FALSE(b->Tick());
  EXPECT_FALSE(b->Tick());
  b->SetDone();
  EXPECT_TRUE(b->Tick());
  EXPECT_EQ(b->GetBehaviourState(), BehaviourState::DONE);
}

TEST(BehaviourTest, Interrupt) {
  auto b = make<MockBehaviour>();

  {
    ::testing::InSequence s;
    EXPECT_CALL(*b, OnStart).Times(1);
    EXPECT_CALL(*b, OnTick).Times(2);
    EXPECT_CALL(*b, OnStop).Times(1);
  }

  EXPECT_FALSE(b->Tick());
  EXPECT_FALSE(b->Tick());
  b->Interrupt();
  EXPECT_TRUE(b->Tick());
  EXPECT_EQ(b->GetBehaviourState(), BehaviourState::INTERRUPTED);
}

TEST(BehaviourTest, Timeout) {
  auto b = make<MockBehaviour>();
  b->WithTimeout(10_ms);

  {
    ::testing::InSequence s;
    EXPECT_CALL(*b, OnStart).Times(1);
    EXPECT_CALL(*b, OnTick).Times(2);
    EXPECT_CALL(*b, OnStop).Times(1);
  }

  EXPECT_FALSE(b->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(6));
  EXPECT_FALSE(b->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(6));
  EXPECT_TRUE(b->Tick());
}

TEST(SequentialBehaviourTest, InheritsControls) {
  HasBehaviour a, b;
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>();
  b1->Controls(&a);
  b2->Controls(&a);
  b2->Controls(&b);

  auto chain = b1 << b2;
  ASSERT_EQ(chain->GetControlled().count(&a), 1);
  ASSERT_EQ(chain->GetControlled().count(&b), 1);
}

TEST(SequentialBehaviourTest, Sequence) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>(),
       b3 = make<MockBehaviour>(), b4 = make<MockBehaviour>();
  auto chain = b1 << b2 << b3 << b4;

  {
    ::testing::InSequence s;
    EXPECT_CALL(*b1, OnStart).Times(1);
    EXPECT_CALL(*b1, OnTick).Times(2);
    EXPECT_CALL(*b1, OnStop).Times(1);
    EXPECT_CALL(*b2, OnStart).Times(1);
    EXPECT_CALL(*b2, OnTick).Times(1);
    EXPECT_CALL(*b2, OnStop).Times(1);
    EXPECT_CALL(*b3, OnStart).Times(1);
    EXPECT_CALL(*b3, OnTick).Times(1);
    EXPECT_CALL(*b3, OnStop).Times(1);
  }

  EXPECT_FALSE(chain->Tick());
  EXPECT_FALSE(chain->Tick());
  b1->SetDone();
  EXPECT_FALSE(chain->Tick());
  b2->Interrupt();
  EXPECT_FALSE(chain->Tick());
  chain->Interrupt();
  EXPECT_TRUE(chain->Tick());
  ASSERT_EQ(b1->GetBehaviourState(), BehaviourState::DONE);
  ASSERT_EQ(b2->GetBehaviourState(), BehaviourState::INTERRUPTED);
  ASSERT_EQ(b3->GetBehaviourState(), BehaviourState::INTERRUPTED);
  ASSERT_EQ(b4->GetBehaviourState(), BehaviourState::INTERRUPTED);
}

TEST(ConcurrentBehaviourTest, InheritsControls) {
  HasBehaviour a, b;
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>(),
       b3 = make<MockBehaviour>();
  b1->Controls(&a);
  b2->Controls(&b);
  b3->Controls(&a);

  auto chain1 = b1 & b2;
  ASSERT_EQ(chain1->GetControlled().count(&a), 1);
  ASSERT_EQ(chain1->GetControlled().count(&b), 1);

  EXPECT_THROW(b1 | b3, DuplicateControlException);
}

TEST(ConcurrentBehaviourTest, Race) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>();
  b1->SetPeriod(20_ms);
  b2->SetPeriod(10_ms);

  auto chain = b1 | b2;
  chain->SetPeriod(1_s);  // Silence Period warning

  EXPECT_CALL(*b1, OnStart).Times(1);
  EXPECT_CALL(*b2, OnStart).Times(1);
  EXPECT_CALL(*b1, OnTick).Times(::testing::Between(4, 6));
  EXPECT_CALL(*b2, OnTick).Times(::testing::Between(9, 11));
  EXPECT_CALL(*b1, OnStop).Times(1);
  EXPECT_CALL(*b2, OnStop).Times(1);

  ASSERT_FALSE(chain->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  b1->SetDone();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_TRUE(chain->Tick());
  EXPECT_EQ(b1->GetBehaviourState(), BehaviourState::DONE);
  EXPECT_EQ(b2->GetBehaviourState(), BehaviourState::INTERRUPTED);
}

TEST(ConcurrentBehaviourTest, All) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>();
  b1->SetPeriod(20_ms);
  b2->SetPeriod(10_ms);

  auto chain = b1 & b2;
  chain->SetPeriod(1_s);  // Silence Period warning

  EXPECT_CALL(*b1, OnStart).Times(1);
  EXPECT_CALL(*b2, OnStart).Times(1);
  EXPECT_CALL(*b1, OnTick).Times(::testing::Between(1, 6));
  EXPECT_CALL(*b2, OnTick).Times(::testing::Between(1, 16));
  EXPECT_CALL(*b1, OnStop).Times(1);
  EXPECT_CALL(*b2, OnStop).Times(1);

  ASSERT_FALSE(chain->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  b1->SetDone();
  ASSERT_FALSE(chain->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  b2->SetDone();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ASSERT_TRUE(chain->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  EXPECT_EQ(b1->GetBehaviourState(), BehaviourState::DONE);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  EXPECT_EQ(b2->GetBehaviourState(), BehaviourState::DONE);
}

TEST(ConcurrentBehaviourTest, Until) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>();
  auto chain = b1->Until(b2);

  EXPECT_CALL(*b1, OnStart).Times(1);
  EXPECT_CALL(*b1, OnTick).Times(::testing::AtLeast(1));
  EXPECT_CALL(*b1, OnStop).Times(1);
  EXPECT_CALL(*b2, OnStart).Times(1);
  EXPECT_CALL(*b2, OnTick).Times(::testing::AtLeast(1));
  EXPECT_CALL(*b2, OnStop).Times(1);

  ASSERT_FALSE(chain->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_TRUE(b1->IsRunning());
  ASSERT_TRUE(b2->IsRunning());

  // TODO: Need to add a way to have SetDone interrupt the wait on
  // the thread. E.g. SleepOrUntilDone
  b2->SetDone();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_TRUE(chain->Tick());
  ASSERT_FALSE(b1->IsRunning());
  ASSERT_FALSE(b2->IsRunning());
}

TEST(WaitForTest, Waits) {
  bool v = false;
  auto b = make<WaitFor>([&v]() { return v; });

  ASSERT_FALSE(b->Tick());
  ASSERT_FALSE(b->Tick());
  v = true;
  ASSERT_TRUE(b->Tick());
  ASSERT_EQ(b->GetBehaviourState(), BehaviourState::DONE);
}

TEST(WaitTimeTest, Waits) {
  auto b = make<WaitTime>(20_ms);

  ASSERT_FALSE(b->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(11));
  ASSERT_FALSE(b->Tick());
  std::this_thread::sleep_for(std::chrono::milliseconds(11));
  ASSERT_TRUE(b->Tick());
  ASSERT_EQ(b->GetBehaviourState(), BehaviourState::DONE);
}

TEST(IfTest, Then) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>();

  auto chain = make<If>(true)->Then(b1)->Else(b2);

  EXPECT_CALL(*b1, OnStart).Times(1);
  EXPECT_CALL(*b1, OnTick).Times(2);

  chain->Tick();
  chain->Tick();
}

TEST(IfTest, Else) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>();

  auto chain = make<If>(false)->Then(b1)->Else(b2);

  EXPECT_CALL(*b2, OnStart).Times(1);
  EXPECT_CALL(*b2, OnTick).Times(2);

  chain->Tick();
  chain->Tick();
}

TEST(SwitchTest, Int) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>(),
       b3 = make<MockBehaviour>();

  auto chain = make<Switch<int>>(1)->When(0, b1)->When(1, b2)->When(2, b3);

  EXPECT_CALL(*b2, OnStart).Times(1);
  EXPECT_CALL(*b2, OnTick).Times(2);

  chain->Tick();
  chain->Tick();
}

TEST(SwitchTest, Decide) {
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>(),
       b3 = make<MockBehaviour>();

  auto chain = make<Decide>()
                   ->When([]() { return true; }, b1)
                   ->When([]() { return false; }, b2)
                   ->When([]() { return false; }, b3);

  EXPECT_CALL(*b1, OnStart).Times(1);
  EXPECT_CALL(*b1, OnTick).Times(2);

  chain->Tick();
  chain->Tick();
}

TEST(BehaviourTest, FullChain) {
  BehaviourScheduler s;
  MockSystem a, b;
  auto b1 = make<MockBehaviour>(), b2 = make<MockBehaviour>(),
       b3 = make<MockBehaviour>(), b4 = make<MockBehaviour>();

  b1->Controls(&a);
  b2->Controls(&b);
  b3->Controls(&a);

  b1->SetPeriod(10_ms);
  b2->SetPeriod(50_ms);
  b3->SetPeriod(25_ms);
  b4->SetPeriod(1_s / 75.0);

  EXPECT_CALL(*b1, OnStart).Times(1);
  EXPECT_CALL(*b2, OnStart).Times(1);
  EXPECT_CALL(*b3, OnStart).Times(1);
  EXPECT_CALL(*b4, OnStart).Times(1);

  EXPECT_CALL(*b1, OnStop).Times(1);
  EXPECT_CALL(*b2, OnStop).Times(1);
  EXPECT_CALL(*b3, OnStop).Times(1);
  EXPECT_CALL(*b4, OnStop).Times(1);

  EXPECT_CALL(*b1, OnTick).Times(::testing::Between(9, 11));   // 100ms @ 100Hz
  EXPECT_CALL(*b2, OnTick).Times(::testing::Between(5, 7));    // 300ms @ 20Hz
  EXPECT_CALL(*b3, OnTick).Times(::testing::Between(4, 6));    // 100ms @ 50Hz
  EXPECT_CALL(*b4, OnTick).Times(::testing::Between(22, 23));  // 300ms @ 75Hz

  auto chain = ((b1 << b3) & b2) | b4;

  s.Tick();
  s.Register(&a);
  s.Register(&b);

  ASSERT_EQ(a.GetActiveBehaviour(), nullptr);
  ASSERT_EQ(b.GetActiveBehaviour(), nullptr);

  s.Schedule(chain);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  s.Tick();

  ASSERT_EQ(a.GetActiveBehaviour(), chain);
  ASSERT_EQ(b.GetActiveBehaviour(), chain);

  ASSERT_TRUE(b1->IsRunning());
  ASSERT_TRUE(b2->IsRunning());
  ASSERT_FALSE(b3->IsRunning());
  ASSERT_TRUE(b4->IsRunning());

  b1->SetDone();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  s.Tick();

  ASSERT_FALSE(b1->IsRunning());
  ASSERT_TRUE(b2->IsRunning());
  ASSERT_TRUE(b3->IsRunning());
  ASSERT_TRUE(b4->IsRunning());

  b3->SetDone();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  s.Tick();

  ASSERT_FALSE(b1->IsRunning());
  ASSERT_TRUE(b2->IsRunning());
  ASSERT_FALSE(b3->IsRunning());
  ASSERT_TRUE(b4->IsRunning());

  b4->SetDone();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  s.Tick();

  ASSERT_FALSE(b1->IsRunning());
  ASSERT_FALSE(b2->IsRunning());
  ASSERT_FALSE(b3->IsRunning());
  ASSERT_FALSE(b4->IsRunning());

  ASSERT_EQ(b2->GetBehaviourState(), BehaviourState::INTERRUPTED);
  ASSERT_EQ(b4->GetBehaviourState(), BehaviourState::DONE);

  ASSERT_EQ(a.GetActiveBehaviour(), nullptr);
  ASSERT_EQ(b.GetActiveBehaviour(), nullptr);
}