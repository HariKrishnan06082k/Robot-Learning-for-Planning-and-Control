import torch

# Type hints.
from typing import List, Tuple
from torch import Tensor


def create_tensor_of_pi(M: int, N: int) -> Tensor:
    """
    Returns a Tensor of shape (M, N) filled entirely with the value 3.14

    Args:
        M, N: Positive integers giving the shape of Tensor to create

    Returns:
        x: A tensor of shape (M, N) filled with the value 3.14
    """
    x = None
    # --- Your code here
    x = torch.full((M,N),3.14)
    # ---
    return x


def slice_indexing_practice(x: Tensor) -> Tuple[Tensor, Tensor, Tensor, Tensor]:
    """
    Given a two-dimensional tensor x, extract and return several subtensors to
    practice with slice indexing. Each tensor should be created using a single
    slice indexing operation.

    The input tensor should not be modified.

    Args:
        x: Tensor of shape (M, N) -- M rows, N columns with M >= 3 and N >= 5.

    Returns:
        A tuple of:
        - last_row: Tensor of shape (N,) giving the last row of x. It should be
          a one-dimensional tensor.
        - third_col: Tensor of shape (M, 1) giving the third column of x. It
          should be a two-dimensional tensor.
        - first_two_rows_three_cols: Tensor of shape (2, 3) giving the data in
          the first two rows and first three columns of x.
        - even_rows_odd_cols: Two-dimensional tensor containing the elements in
          the even-valued rows and odd-valued columns of x.
    """
    assert x.shape[0] >= 3
    assert x.shape[1] >= 5
    last_row = None
    third_col = None
    first_two_rows_three_cols = None
    even_rows_odd_cols = None
    # --- Your code here
    (M,N) = x.shape
    last_row = x[M-1]
    third_col = x[:,2:3]
    first_two_rows_three_cols = x[:2,:3]
    even_rows_odd_cols = x[::2,1::2]
    # ---

    out = (
        last_row,
        third_col,
        first_two_rows_three_cols,
        even_rows_odd_cols,
    )
    return out


def slice_assignment_practice(x: Tensor) -> Tensor:
    """
    Given a two-dimensional tensor of shape (M, N) with M >= 4, N >= 6, mutate
    its first 4 rows and 6 columns so they are equal to:

    [0 1 2 2 2 2]
    [0 1 2 2 2 2]
    [3 4 3 4 5 5]
    [3 4 3 4 5 5]

    Note: the input tensor shape is not fixed to (4, 6).

    Your implementation must obey the following:
    - You should mutate the tensor x in-place and return it
    - You should only modify the first 4 rows and first 6 columns; all other
      elements should remain unchanged
    - You may only mutate the tensor using slice assignment operations, where
      you assign an integer to a slice of the tensor
    - You must use <= 6 slicing operations to achieve the desired result

    Args:
        x: A tensor of shape (M, N) with M >= 4 and N >= 6

    Returns:
        x
    """
    # --- Your code here
    x[0:2,0] = 0
    x[0:2,1] = 1
    x[0:2,2:6] = 2
    x[2:4,0:3:2] = 3
    x[2:4,1:4:2] = 4
    x[2:4,4:6] = 5
    # ---
    return x


def shuffle_cols(x: Tensor) -> Tensor:
    """
    Re-order the columns of an input tensor as described below.

    Your implementation should construct the output tensor using a single
    integer array indexing operation. The input tensor should not be modified.

    Args:
        x: A tensor of shape (M, N) with N >= 3

    Returns:
        A tensor y of shape (M, 4) where:
        - The first two columns of y are copies of the first column of x
        - The third column of y is the same as the third column of x
        - The fourth column of y is the same as the second column of x
    """
    y = None
    # --- Your code here
    idx = [0,0,2,1]
    y = x[:,idx]
    # ---
    return y


def reverse_rows(x: Tensor) -> Tensor:
    """
    Reverse the rows of the input tensor.

    Your implementation should construct the output tensor using a single
    integer array indexing operation. The input tensor should not be modified.

    Args:
        x: A tensor of shape (M, N)

    Returns:
        y: Tensor of shape (M, N) which is the same as x but with the rows
            reversed - the first row of y should be equal to the last row of x,
            the second row of y should be equal to the second to last row of x,
            and so on.
    """
    y = None
    # --- Your code here
    idx1 = torch.arange(x.shape[0]-1,-1,-1)
    y = x[idx1]
    # ---
    return y


def reshape_practice(x: Tensor) -> Tensor:
    """
    Given an input tensor of shape (24,), return a reshaped tensor y of shape
    (3, 8) such that

    y = [[x[0], x[1], x[2],  x[3],  x[12], x[13], x[14], x[15]],
         [x[4], x[5], x[6],  x[7],  x[16], x[17], x[18], x[19]],
         [x[8], x[9], x[10], x[11], x[20], x[21], x[22], x[23]]]

    You must construct y by performing a sequence of reshaping operations on
    x (view, t, transpose, permute, contiguous, reshape, etc). The input
    tensor should not be modified.

    Args:
        x: A tensor of shape (24,)

    Returns:
        y: A reshaped version of x of shape (3, 8) as described above.
    """
    y = None
    # --- Your cody e here
    y = x.reshape(2,3,4)
    y = y.transpose(1,0).reshape(3,8)
    # ---
    return y


def batched_matrix_multiply(x: Tensor, y: Tensor) -> Tensor:
    """
    Perform batched matrix multiplication between the tensor x of shape
    (B, N, M) and the tensor y of shape (B, M, P).

    This implementation should use no explicit Python loops (including
    comprehensions).

    Hint: torch.bmm

    Args:
        x: Tensor of shaper (B, N, M)
        y: Tensor of shape (B, M, P)

    Returns:
        z: Tensor of shape (B, N, P) where z[i] of shape (N, P) is the result
            of matrix multiplication between x[i] of shape (N, M) and y[i] of
            shape (M, P). The output z should have the same dtype as x.
    """
    z = None
    # --- Your code here
    z  = torch.bmm(x,y)
    # ---
    return z


def compute_scalar_function_and_grad(x: Tensor) -> Tensor:
    """
        Compute the function y = 3*x^2 and compute the gradient using PyTorch autograd

        You only need to return y, if you have successfully computed the gradient using PyTorch autograd the
        gradient will be stored in x.grad
    Args:
        x: Tensor of shape (1).

    Returns:
        y: Tensor of shape (1) as described above. It should have the same
            dtype as the input x.
    """
    y = None
    x = x.requires_grad_(True)
    # --- Your code here
    y = 3 * torch.pow(x,2)
    y.backward()
    # ---
    return y


def compute_vector_function_and_grad(x: Tensor) -> Tensor:
    """
        Compute the vector function
            y1 = cos(2*x1 + x2)
            y2 = sin(2*x2 - x1)

         and compute the gradient using PyTorch autograd

        You only need to return y, if you have successfully computed the gradient using PyTorch autograd the
        gradient will be stored in x.grad
    Args:
        x: Tensor of shape (2).

    Returns:
        y: Tensor of shape (2) as described above. It should have the same
            dtype as the input x.
    """
    y = None
    x = x.requires_grad_(True)
    # --- Your code here
    y1 = torch.cos(2*x[0] + x[1])
    y2 = torch.sin(2*x[1] - x[0])
    y1.backward()
    y2.backward()
    y = torch.tensor([y1,y2],dtype = x.dtype)
    # ---
    return y


def compute_scalar_function_and_partial_grad(x: Tensor, y: Tensor) -> Tensor:
    """
        Compute the vector function
            z = x^0.5 * y

         and compute the gradient using PyTorch autograd ONLY with respect to x - we do not want to compute the
         gradient with respect to y

        You only need to return y, if you have successfuly computed the gradient using PyTorch autograd the
        gradient will be stored in x.grad BUT the gradient stored in y.grad should be None
    Args:
        x: Tensor of shape (1).
        y: Tensor of shape (1).

    Returns:
        z: Tensor of shape (1) as described above. It should have the same
            dtype as the input x.
    """
    z = None
    # --- Your code here
    x = x.requires_grad_(True)
    z = torch.pow(x,0.5) * y
    z.backward()
    # ---
    return z


def compute_forward_kinematics(thetas: torch.Tensor) -> torch.Tensor:
    """
    Compute the forward kinematics of the robot configuration given by theta
    Args:
        thetas: Pytorch Tensor of shape (2,) containing the robot joints
    Returns:
        x: Pytorch Tensor of shape (2,) containing the end-effector position

    """
    L1 = 2
    L2 = 1
    x = None
    # --- Your code here
    x = torch.zeros(2)
    x[0] = L1 * torch.cos(thetas[0]) + L2 * torch.cos(thetas[0] + thetas[1])
    x[1] = L1 * torch.sin(thetas[0]) + L2 * torch.sin(thetas[0] + thetas[1])
    # ---
    return x


def compute_jacobian(thetas: torch.Tensor) -> torch.Tensor:
    """
    Compute the manipulator Jacobian
    Args:
        thetas: Pytorch Tensor of shape (2,) containing the robot joints
    Returns:
        J: Pytorch Tensor of shape (2,2) containing the end-effector position

    """
    J = None
    # --- Your code here
    J = torch.autograd.functional.jacobian(compute_forward_kinematics,thetas)
    # ---
    return J
