
#ifndef _BLOCK_MATRIX_H
#define _BLOCK_MATRIX_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace BlockMatrix
{

// Types
template<class Type>
class Matrix : public Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>
{
public:
    void resize(int num_rows, int num_cols)
    {
        if(num_rows < 0 || num_cols < 0)
            throw;

        Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>::resize(num_rows, num_cols);

        for(int row=0; row<num_rows; row++)
            for(int col=0; col<num_cols; col++)
                this->operator()(row, col).resize(0, 0);

        return;
    }

    Matrix& operator=(const Matrix& other)
    {
        // protect against invalid self-assignment
        if (this != &other)
        {
            // Resize
            this->resize(other.rows(), other.cols());
            // Copy Values of each block
            for(int row=0; row<this->rows(); row++)
                for(int col=0; col<this->cols(); col++)
                    this->operator()(row,col)=other(row,col);
        }
        // by convention, always return *this
        return *this;
    }

    Matrix transpose()
    {
        Matrix out;

        // Resize
        this->resize(this->cols(), this->rows());
        // Copy Values of each block
        for(int row=0; row<this->rows(); row++)
            for(int col=0; col<this->cols(); col++)
            {
                if(this->operator()(col, row).cols()>0 && this->operator()(col, row).rows()>0)
                {
                    out(row,col).resize(this->operator()(col, row).cols(), this->operator()(col, row).rows());
                    out(row,col)=this->operator()(col, row).transpose();
                }
            }

        return out;
    }


};

using MatrixSparse = Matrix<Eigen::SparseMatrix<double>>;

using MatrixDense = Matrix<Eigen::MatrixXd>;



// Operators
//template<class Type1, class Type2, class TypeR>
//Matrix<TypeR> operator*(const Matrix<Type1> &prod1, const Matrix<Type2> &prod2)
MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2)
{
    //Matrix<TypeR> product_matrix;
    MatrixSparse product_matrix;

    // Check multiplication sizes
    if(prod1.cols() != prod2.rows())
        throw;

    // Resize
    product_matrix.resize(prod1.rows(), prod2.cols());

    // Multiplication
    for(int i=0; i<prod1.rows(); i++)
        for(int j=0; j<prod2.cols(); j++)
        {
            // First pass -> checks and dimension of the block
            int row_block=-1;
            int col_block=-1;
            for(int k=0; k<prod1.cols(); k++)
            {
                // Check prod1
                if(prod1(i,k).cols() == 0 || prod1(k,j).rows() == 0)
                    continue;

                // Check prod2
                if(prod2(i,k).cols() == 0 || prod2(k,j).rows() == 0)
                    continue;

                // Check prod1 * prod2
                if(prod1(i,k).cols() != prod2(k,j).rows())
                    throw;

                // Rows and cols
                if(row_block<0)
                    row_block=prod1(k,j).rows();
                if(col_block<0)
                    col_block=prod2(k,j).cols();

                // Check
                if(row_block != prod1(k,j).rows())
                    throw;

                if(col_block != prod2(k,j).cols())
                    throw;
            }

            // Dimensions
            if(row_block < 0 || col_block < 0)
                continue;

            // Resize and set zero
            product_matrix(i,j).resize(row_block, col_block);
            product_matrix(i,j).setZero();

            // Second pass -> Multiplication
            for(int k=0; k<prod1.cols(); k++)
            {
                if(prod1(i,k).cols() == row_block && prod2(k,j).rows() == col_block)
                    product_matrix(i,j)+= prod1(i,k)*prod2(k,j);

            }
        }


    // End
    return product_matrix;
}


//template<>
//MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2);



Eigen::SparseMatrix<double> convertToEigenSparse(const MatrixSparse& in)
{
    Eigen::SparseMatrix<double> out;


    // TODO


    return out;
}


Eigen::MatrixXd convertToEigenDense(const MatrixSparse& in)
{
    Eigen::MatrixXd out;


    // TODO


    return out;
}


}







#endif
