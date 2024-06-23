#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>

// Signal handler function for user-defined signal (e.g., SIGUSR1)
void user_signal_handler(int signo) {
    if (signo == SIGUSR1) {
        printf("Received user-defined signal SIGUSR1.\n");
    }
}

// Function declarations
void function1(void) {
    for(;;)
   {
        printf("Function 1 is called.\n");
        sleep(1); // Simulate some work
    }
}

void function2(void) {
    printf("Function 2 is called.\n");
    sleep(1); // Simulate some work
}

void function3(void) {
    printf("Function 3 is called.\n");
    sleep(1); // Simulate some work
}

void function4(void) {
for(;;)
   {
    printf("Function 4 is called.\n");
    sleep(1); // Simulate some work
   }
}

// Thread function wrappers
void* thread_func1(void* arg) {
    function1();
    return NULL;
}

void* thread_func2(void* arg) {
    function2();
    return NULL;
}

void* thread_func3(void* arg) {
    function3();
    return NULL;
}

void* thread_func4(void* arg) {
    function4();
    return NULL;
}

// Main task function
void* main_task(void* arg) {
    pthread_t threads[4];
    int ret;

    // Create threads
    ret = pthread_create(&threads[0], NULL, thread_func1, NULL);
    if (ret != 0) {
        printf("Failed to create thread 1\n");
        return NULL;
    }
    
    ret = pthread_create(&threads[1], NULL, thread_func2, NULL);
    if (ret != 0) {
        printf("Failed to create thread 2\n");
        return NULL;
    }
    
    ret = pthread_create(&threads[2], NULL, thread_func3, NULL);
    if (ret != 0) {
        printf("Failed to create thread 3\n");
        return NULL;
    }
    
    ret = pthread_create(&threads[3], NULL, thread_func4, NULL);
    if (ret != 0) {
        printf("Failed to create thread 4\n");
        return NULL;
    }

    // Simulate some condition to send the signal
    sleep(2); // Wait for a while before sending the signal
    if (0) { // Replace with your condition
        printf("Sending SIGUSR1 to self.\n");
        kill(getpid(), SIGUSR1);
    }

    // Wait for threads to complete
    for (int i = 0; i < 4; i++) {
        pthread_join(threads[i], NULL);
    }

    printf("All threads completed.\n");
     while(1){
        usleep(1000);
    }
}

// Main application
int main(int argc, FAR char *argv[]) {
    // pthread_t task,task1;
    int ret;

    // Set up the user-defined signal handler for SIGUSR1
    if (signal(SIGUSR1, user_signal_handler) == SIG_ERR) {
        printf("Error setting up user-defined signal handler.\n");
        return -1;
    }

    // Create the main task
    ret = task_create("main task",1, 2048, thread_func1, NULL);
    if (ret != 0) {
        printf("Failed to create the main task\n");
        return -1;
    }
   ret = task_create("sub task",100, 2048, thread_func4, NULL);
    if (ret != 0) {
        printf("Failed to create the main task\n");
        return -1;
    }
   
    // Wait for the main task to complete
    // pthread_join(task, NULL);
    
    return 0;
}
