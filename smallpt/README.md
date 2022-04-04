# introduce

- [nanopt.cpp](./nanopt.cpp): simplfy `smallpt.cpp`

- [smallpt.cpp](./smallpt.cpp): original version

- [smallpt_format.cpp](./smallpt_format.cpp): format `smallpt.cpp`

- [smallpt_comment.cpp](./smallpt_comment.cpp): comment `smallpt_format.cpp`

  - [smallpt_kernel.cpp](./smallpt_kernel.cpp): run `smallpt_comment.cpp` on GPU

- [smallpt_rewrite.cpp](./smallpt_rewrite.cpp): rewrite `smallpt_comment.cpp` to pbrt style
 
- [ky.cpp](../ky.cpp): enhanced version of `smallpt_rewrite.cpp` 

# recommended reading order

&emsp;&emsp; [ [nanopt.cpp](./nanopt.cpp) => ] [smallpt_comment.cpp](./smallpt_comment.cpp) => [smallpt_rewrite.cpp](./smallpt_rewrite.cpp) => [ky.cpp](../ky.cpp)