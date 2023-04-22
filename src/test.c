#include <unistd.h>

int main() {
	execve("ls", NULL, NULL);
	return 1;
}
