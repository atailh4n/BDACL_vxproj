# BDACL (Branch and Divergence Aware Computing Layer)
Actually it is not for now. _Not yet implemented_

Copyright 2025 Ata Ilhan Kokturk.

# DAGL (Dependency Aware Graph Layer)
## 📌 Example Usage of DATask_Scheduler

This example demonstrates how to define and schedule dependent tasks using `BDACL::DATask_Scheduler`. A logger is used for synchronized output, and `std::promise/future` is used to wait for the final task's completion.
## 🧱 Prerequisites

Make sure you have:

1. Initialized your thread pool with an appropriate number of threads.
2. Use `constexpr` for statically known values to maximize performance at compile-time.
3. A thread-safe logger (`ThreadSafeLogger`) available.
4. Task dependencies defined via `addDependant`.

## ✅ Sample Code
```cpp
#include <future>
#include "BDACL.hpp"
#include "DAGL.hpp"
...
constexpr int THREADS = 4;
...
// Define the logger and the task scheduler.
void runChainedTasksExample() {
	BDACL::ThreadSafeLogger logger;
	BDACL::DATask_Scheduler daTaskSch;
	
	// Promise/future used to wait for the last task to finish.
	std::promise<void> promise;
	std::future<void> future = promise.get_future();
	
	// Initialize the thread pool with THREADS number of workers.
	daTaskSch.initSchedulerPool(THREADS);
	
	// Define two tasks
	BDACL::DATask task1, task2;
	
	// Optional: Scoping helps visually separate task definitions
	// Task 1: Just logs "Hello World"
	{
		task1.job = [&logger] {
			logger.log("Hello World!"); // Logger prevents race conditions
		};
	}
	// Task 2: Depends on task1, then logs a message and notifies main thread
	{
		task2.job = [&logger, &promise] {
			logger.log("This one must wait for task1!");
			promise.set_value(); // Notify that the task chain is complete
		};
	}
	
	// Define dependency: task2 will not run until task1 is complete
	daTaskSch.addDependant(&task1, &task2); // task1 → task2
	
	// Start the scheduler by adding only root/independent tasks
	daTaskSch.addTask(&task1);
	
	// Wait until task2 signals completion
	future.get();
	
	// Gracefully shutdown the scheduler (joins all threads)
	daTaskSch.shutdown();
}
```

## 📌 Notes

- Only add root tasks (those without dependencies) using `addTask()`.
- Dependent tasks will automatically be enqueued by the scheduler once their parents complete.
- `ThreadSafeLogger` ensures clean, ordered output in multi-threaded environments.
- You can chain more tasks using `addDependant(&parent, &child)` as needed.
- If you don't use `std::promise/future`, the `shutdown` method will **not wait** for all tasks to complete.
  This may result in **undefined behavior** (UB), especially if dependent tasks are still running.

## 🔄 Visual Flow

`[ task1 ] --(on complete)--> [ task2 ] --(on complete)--> promise.set_value() --(after future.get())--> shutdown()`
