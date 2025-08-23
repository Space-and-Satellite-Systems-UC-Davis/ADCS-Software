#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include "pointdish.h"

// How many people we may hold in queue
#define BACKLOG 10

int start_listening() {
    struct addrinfo hints;
    struct addrinof *result;

    memset(&hints, 0, sizeof hints); // make sure the struct is empty

    // Settings
    hints.ai_family = AF_UNSPEC;     // Don't care about v4 vs v6
    hints.ai_socktype = SOCK_STREAM; // TCP stream socket
    hints.ai_flags = AI_PASSIVE;     // fill in my IP for me

    // Get address or fail
    int status;
    if ((status = getaddrinfo(NULL, "http", &hints, &result)) != 0) {
        fprintf(stderr, "gai error: %s\n", gai_strerror(status));
        exit(1);
    }

    int sockfd;
    // Walk linked list to find suitable address
    for(struct addrinfo *p = result; p != NULL; p = p->ai_next) {
        //Create socket
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
            p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }

        //See if socket used
        int yes = 1; // Share socket if needed
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                sizeof(int)) == -1) {
            perror("setsockopt");
            exit(1);
        }

        //Bind if listening
        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("server: bind");
            continue;
        }

        break;
    }
    freeaddrinfo(result); // Done with struct;

    int listen_result = listen(sockfd, BACKLOG);

    //If error return error. Otherwise return socket file descriptor
    int function_result = listen_result;
    if (!function_result) function_result = sockfd;

    return function_result;
}

// Code is written with linux in mind. Compile accordingly
// Our Pi isn't going to be running windows.
int main() {
    printf("Starting server\n");
    
    int sockfd = start_listening();
    if (!sockfd) {
        exit(1); // error
    }

    printf("[SERVER]: Listening on 80\n");

    struct sockaddr_storage their_addr;
    socklen_t addr_size;
    int new_fd; // fd for the client
    addr_size = sizeof their_addr;
    new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &addr_size);

    char *msg = "Hiiiiiiiii !!!\n";
    send(new_fd, msg, strlen(msg), 0);

    // while (true) {
    //     server();
    // }

    return 0;
}