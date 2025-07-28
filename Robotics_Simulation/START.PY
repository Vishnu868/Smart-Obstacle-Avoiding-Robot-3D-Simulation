def solid_butterfly(n):
    # Upper half
    for i in range(1, n + 1):
        print("*" * i + " " * (2 * (n - i)) + "*" * i)
    
    # Lower half
    for i in range(n - 1,0, -1):
        print("*" * i + " " * (2 * (n - i)) + "*" * i)

# Input from user
n = int(input("Enter the number of rows: "))
solid_butterfly(n)
