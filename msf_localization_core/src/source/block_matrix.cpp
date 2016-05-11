
#include "msf_localization_core/block_matrix.h"


namespace BlockMatrix
{

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



Eigen::SparseMatrix<double> convertToEigenSparse(MatrixSparse& in)
{
    Eigen::SparseMatrix<double> out;

    // Analyse
    if(in.analyse())
        throw;

    // Resize out
    out.resize(in.getTotalRowsSize(), in.getTotalColsSize());

    //std::cout<<"rows="<<out.rows()<<"; cols="<<out.cols()<<std::endl;

    // First pass to get the size of non-zero coefficient
    int size_non_zero_coef=0;
    for(int i=0; i<in.rows(); i++)
        for(int j=0; j<in.cols(); j++)
            size_non_zero_coef+=in.nonZeros();

    // Reserve out
    out.reserve(size_non_zero_coef);

    //std::cout<<"size_non_zero_coef="<<size_non_zero_coef<<std::endl;

    // Second pass to get the non-zero coeffs
    std::vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(size_non_zero_coef);
    for(int i=0; i<in.rows(); i++)
    {
        for(int j=0; j<in.cols(); j++)
        {
            for (int k=0; k<in(i,j).outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(in(i,j),k); it; ++it)
                {
                    triplet_list.push_back(Eigen::Triplet<double>(it.row(),it.col(),it.value()));
                }
            }
        }
    }

    // Set out
    out.setFromTriplets(triplet_list.begin(), triplet_list.end());

    // End
    return out;
}


Eigen::MatrixXd convertToEigenDense(const MatrixSparse& in)
{
    Eigen::MatrixXd out;


    // TODO


    return out;
}


std::vector< Eigen::Triplet<double> > getVectorEigenTripletFromEigenDense(const Eigen::MatrixXd& in, int row_offset, int col_offset)
{
    std::vector< Eigen::Triplet<double> > triplets;
    for(int i=0; i<in.rows(); i++)
        for(int j=0; j<in.cols(); j++)
            triplets.push_back(Eigen::Triplet<double>(i+row_offset,j+col_offset,in(i,j)));

    return triplets;
}

int insertVectorEigenTripletFromEigenDense(std::vector< Eigen::Triplet<double> >& triplets, const Eigen::MatrixXd& in, int row_offset, int col_offset)
{
    for(int i=0; i<in.rows(); i++)
        for(int j=0; j<in.cols(); j++)
            triplets.push_back(Eigen::Triplet<double>(i+row_offset,j+col_offset,in(i,j)));
    return 0;
}



}
