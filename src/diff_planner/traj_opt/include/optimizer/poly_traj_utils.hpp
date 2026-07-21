#pragma once

#include "root_finder.hpp"

#include <iostream>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace poly_traj
{

    // Polynomial order and trajectory dimension are fixed here
    typedef Eigen::Matrix<double, 3, 6> CoefficientMatrix;
    typedef Eigen::Matrix<double, 3, 5> VelocityCoefficientMatrix;
    typedef Eigen::Matrix<double, 3, 4> AccelerationCoefficientMatrix;

    class Piece
    {
    private:
        double duration_;
        CoefficientMatrix coefficient_matrix_;

    public:
        Piece() = default;

        Piece(double duration, const CoefficientMatrix &coefficient_matrix)
            : duration_(duration), coefficient_matrix_(coefficient_matrix) {}

        inline int GetDimension() const
        {
            return 3;
        }

        inline int GetOrder() const
        {
            return 5;
        }

        inline double GetDuration() const
        {
            return duration_;
        }

        inline const CoefficientMatrix &GetCoefficientMatrix() const
        {
            return coefficient_matrix_;
        }

        inline VelocityCoefficientMatrix GetVelocityCoefficientMatrix() const
        {
            VelocityCoefficientMatrix velocity_coefficient_matrix;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                velocity_coefficient_matrix.col(i) = n * coefficient_matrix_.col(i);
                n++;
            }
            return velocity_coefficient_matrix;
        }

        inline Eigen::Vector3d GetPosition(const double &t) const
        {
            Eigen::Vector3d pos(0.0, 0.0, 0.0);
            double tn = 1.0;
            for (int i = 5; i >= 0; i--)
            {
                pos += tn * coefficient_matrix_.col(i);
                tn *= t;
            }
            return pos;
        }

        inline Eigen::Vector3d GetVelocity(const double &t) const
        {
            Eigen::Vector3d vel(0.0, 0.0, 0.0);
            double tn = 1.0;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                vel += n * tn * coefficient_matrix_.col(i);
                tn *= t;
                n++;
            }
            return vel;
        }

        inline Eigen::Vector3d GetAcceleration(const double &t) const
        {
            Eigen::Vector3d acc(0.0, 0.0, 0.0);
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = 3; i >= 0; i--)
            {
                acc += m * n * tn * coefficient_matrix_.col(i);
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        inline Eigen::Vector3d GetJerk(const double &t) const
        {
            Eigen::Vector3d jer(0.0, 0.0, 0.0);
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = 2; i >= 0; i--)
            {
                jer += l * m * n * tn * coefficient_matrix_.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        inline CoefficientMatrix NormalizePositionCoefficientMatrix() const
        {
            CoefficientMatrix normalized_position_coefficients;
            double t = 1.0;
            for (int i = 5; i >= 0; i--)
            {
                normalized_position_coefficients.col(i) = coefficient_matrix_.col(i) * t;
                t *= duration_;
            }
            return normalized_position_coefficients;
        }

        inline VelocityCoefficientMatrix NormalizeVelocityCoefficientMatrix() const
        {
            VelocityCoefficientMatrix normalized_velocity_coefficients;
            int n = 1;
            double t = duration_;
            for (int i = 4; i >= 0; i--)
            {
                normalized_velocity_coefficients.col(i) = n * coefficient_matrix_.col(i) * t;
                t *= duration_;
                n++;
            }
            return normalized_velocity_coefficients;
        }

        inline AccelerationCoefficientMatrix NormalizeAccelerationCoefficientMatrix() const
        {
            AccelerationCoefficientMatrix normalized_acceleration_coefficients;
            int n = 2;
            int m = 1;
            double t = duration_ * duration_;
            for (int i = 3; i >= 0; i--)
            {
                normalized_acceleration_coefficients.col(i) = n * m * coefficient_matrix_.col(i) * t;
                n++;
                m++;
                t *= duration_;
            }
            return normalized_acceleration_coefficients;
        }

        inline double GetMaxVelocityRate() const
        {
            Eigen::MatrixXd normalized_velocity_coefficients = NormalizeVelocityCoefficientMatrix();
            Eigen::VectorXd coeff = RootFinder::polySqr(normalized_velocity_coefficients.row(0)) +
                                    RootFinder::polySqr(normalized_velocity_coefficients.row(1)) +
                                    RootFinder::polySqr(normalized_velocity_coefficients.row(2));
            int coefficient_count = coeff.size();
            int n = coefficient_count - 1;
            for (int i = 0; i < coefficient_count; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(coefficient_count - 1).squaredNorm() < DBL_EPSILON)
            {
                return 0.0;
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(coefficient_count - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(coefficient_count - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(coefficient_count - 1), l, r,
                                                                          FLT_EPSILON / duration_);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double max_velocity_rate_squared = -INFINITY;
                double temporary_norm_squared;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        temporary_norm_squared = GetVelocity((*it) * duration_).squaredNorm();
                        max_velocity_rate_squared = max_velocity_rate_squared < temporary_norm_squared ? temporary_norm_squared : max_velocity_rate_squared;
                    }
                }
                return sqrt(max_velocity_rate_squared);
            }
        }

        inline double GetMaxAccelerationRate() const
        {
            Eigen::MatrixXd normalized_acceleration_coefficients = NormalizeAccelerationCoefficientMatrix();
            Eigen::VectorXd coeff = RootFinder::polySqr(normalized_acceleration_coefficients.row(0)) +
                                    RootFinder::polySqr(normalized_acceleration_coefficients.row(1)) +
                                    RootFinder::polySqr(normalized_acceleration_coefficients.row(2));
            int coefficient_count = coeff.size();
            int n = coefficient_count - 1;
            for (int i = 0; i < coefficient_count; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(coefficient_count - 1).squaredNorm() < DBL_EPSILON)
            {
                return 0.0;
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(coefficient_count - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(coefficient_count - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(coefficient_count - 1), l, r,
                                                                          FLT_EPSILON / duration_);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double max_acceleration_rate_squared = -INFINITY;
                double temporary_norm_squared;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        temporary_norm_squared = GetAcceleration((*it) * duration_).squaredNorm();
                        max_acceleration_rate_squared = max_acceleration_rate_squared < temporary_norm_squared ? temporary_norm_squared : max_acceleration_rate_squared;
                    }
                }
                return sqrt(max_acceleration_rate_squared);
            }
        }

        inline bool CheckMaxVelocityRate(const double &max_velocity_rate) const
        {
            double max_velocity_rate_squared = max_velocity_rate * max_velocity_rate;
            if (GetVelocity(0.0).squaredNorm() >= max_velocity_rate_squared ||
                GetVelocity(duration_).squaredNorm() >= max_velocity_rate_squared)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd normalized_velocity_coefficients = NormalizeVelocityCoefficientMatrix();
                Eigen::VectorXd coeff = RootFinder::polySqr(normalized_velocity_coefficients.row(0)) +
                                        RootFinder::polySqr(normalized_velocity_coefficients.row(1)) +
                                        RootFinder::polySqr(normalized_velocity_coefficients.row(2));
                double t2 = duration_ * duration_;
                coeff.tail<1>()(0) -= max_velocity_rate_squared * t2;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        inline bool CheckMaxAccelerationRate(const double &max_acceleration_rate) const
        {
            double max_acceleration_rate_squared = max_acceleration_rate * max_acceleration_rate;
            if (GetAcceleration(0.0).squaredNorm() >= max_acceleration_rate_squared ||
                GetAcceleration(duration_).squaredNorm() >= max_acceleration_rate_squared)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd normalized_acceleration_coefficients = NormalizeAccelerationCoefficientMatrix();
                Eigen::VectorXd coeff = RootFinder::polySqr(normalized_acceleration_coefficients.row(0)) +
                                        RootFinder::polySqr(normalized_acceleration_coefficients.row(1)) +
                                        RootFinder::polySqr(normalized_acceleration_coefficients.row(2));
                double t2 = duration_ * duration_;
                double t4 = t2 * t2;
                coeff.tail<1>()(0) -= max_acceleration_rate_squared * t4;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        // GaaiLam
        inline bool ProjectPoint(const Eigen::Vector3d &pt,
                               double &projected_time, Eigen::Vector3d &projected_point)
        {
            // 2*(p-p0)^T * \dot{p} = 0
            auto left_coefficients = GetCoefficientMatrix();
            left_coefficients.col(5) = left_coefficients.col(5) - pt;
            auto right_coefficients = GetVelocityCoefficientMatrix();
            Eigen::VectorXd eq = Eigen::VectorXd::Zero(2 * 5);
            for (int j = 0; j < left_coefficients.rows(); ++j)
            {
                eq = eq + RootFinder::polyConv(left_coefficients.row(j), right_coefficients.row(j));
            }
            double l = -0.0625;
            double r = duration_ + 0.0625;
            while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
            {
                r = 0.5 * (duration_ + 1.0);
            }
            std::set<double> roots =
                RootFinder::solvePolynomial(eq, l, r, 1e-6);
            // std::cout << "# roots: " << roots.size() << std::endl;
            double minimum_distance = -1;
            for (const auto &root : roots)
            {
                // std::cout << "root: " << root << std::endl;
                if (root < 0 || root > duration_)
                {
                    continue;
                }
                if (GetVelocity(root).norm() < 1e-6)
                { // velocity == 0, ignore it
                    continue;
                }
                // std::cout << "find min!" << std::endl;
                Eigen::Vector3d p = GetPosition(root);
                // std::cout << "p: " << p.transpose() << std::endl;
                double distance = (p - pt).norm();
                if (distance < minimum_distance || minimum_distance < 0)
                {
                    minimum_distance = distance;
                    projected_time = root;
                    projected_point = p;
                }
            }
            return minimum_distance > 0;
        }

        inline bool IntersectPlane(const Eigen::Vector3d p,
                                       const Eigen::Vector3d v,
                                       double &projected_time, Eigen::Vector3d &pt) const
        {
            // (pt - p)^T * v = 0
            auto coeff = GetCoefficientMatrix();
            coeff.col(5) = coeff.col(5) - p;
            Eigen::VectorXd eq = coeff.transpose() * v;
            double l = -0.0625;
            double r = duration_ + 0.0625;
            while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
            {
                r = 0.5 * (duration_ + 1.0);
            }
            std::set<double> roots =
                RootFinder::solvePolynomial(eq, l, r, 1e-6);
            for (const auto &root : roots)
            {
                projected_time = root;
                pt = GetPosition(root);
                return true;
            }
            return false;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    class Trajectory
    {
    private:
        typedef std::vector<Piece> PieceVector;
        PieceVector pieces_;

    public:
        Trajectory() = default;

        Trajectory(const std::vector<double> &durations,
                   const std::vector<CoefficientMatrix> &coefficient_matrices)
        {
            int piece_count = std::min(durations.size(), coefficient_matrices.size());
            pieces_.reserve(piece_count);
            for (int i = 0; i < piece_count; i++)
            {
                pieces_.emplace_back(durations[i], coefficient_matrices[i]);
            }
        }

        inline int GetPieceCount() const
        {
            return pieces_.size();
        }

        inline Eigen::VectorXd GetDurations() const
        {
            int piece_count = GetPieceCount();
            Eigen::VectorXd durations(piece_count);
            for (int i = 0; i < piece_count; i++)
            {
                durations(i) = pieces_[i].GetDuration();
            }
            return durations;
        }

        inline double GetTotalDuration() const
        {
            int piece_count = GetPieceCount();
            double total_duration = 0.0;
            for (int i = 0; i < piece_count; i++)
            {
                total_duration += pieces_[i].GetDuration();
            }
            return total_duration;
        }

        inline Eigen::MatrixXd GetPositions() const
        {
            int piece_count = GetPieceCount();
            Eigen::MatrixXd positions(3, piece_count + 1);
            for (int i = 0; i < piece_count; i++)
            {
                positions.col(i) = pieces_[i].GetCoefficientMatrix().col(5);
            }
            positions.col(piece_count) = pieces_[piece_count - 1].GetPosition(pieces_[piece_count - 1].GetDuration());
            return positions;
        }

        inline const Piece &operator[](int i) const
        {
            return pieces_[i];
        }

        inline Piece &operator[](int i)
        {
            return pieces_[i];
        }

        inline void Clear(void)
        {
            pieces_.clear();
            return;
        }

        inline PieceVector::const_iterator begin() const
        {
            return pieces_.begin();
        }

        inline PieceVector::const_iterator end() const
        {
            return pieces_.end();
        }

        inline PieceVector::iterator begin()
        {
            return pieces_.begin();
        }

        inline PieceVector::iterator end()
        {
            return pieces_.end();
        }

        inline void Reserve(const int &n)
        {
            pieces_.reserve(n);
            return;
        }

        inline void EmplaceBack(const Piece &piece)
        {
            pieces_.emplace_back(piece);
            return;
        }

        inline void EmplaceBack(const double &duration,
                                 const CoefficientMatrix &coefficient_matrix)
        {
            pieces_.emplace_back(duration, coefficient_matrix);
            return;
        }

        inline void Append(const Trajectory &traj)
        {
            pieces_.insert(pieces_.end(), traj.begin(), traj.end());
            return;
        }

        inline int LocatePieceIndex(double &t) const
        {
            int piece_count = GetPieceCount();
            int index;
            double duration;
            for (index = 0;
                 index < piece_count &&
                 t > (duration = pieces_[index].GetDuration());
                 index++)
            {
                t -= duration;
            }
            if (index == piece_count)
            {
                index--;
                t += pieces_[index].GetDuration();
            }
            return index;
        }

        inline Eigen::Vector3d GetPosition(double t) const
        {
            int piece_index = LocatePieceIndex(t);
            return pieces_[piece_index].GetPosition(t);
        }

        inline Eigen::Vector3d GetVelocity(double t) const
        {
            int piece_index = LocatePieceIndex(t);
            return pieces_[piece_index].GetVelocity(t);
        }

        inline Eigen::Vector3d GetAcceleration(double t) const
        {
            int piece_index = LocatePieceIndex(t);
            return pieces_[piece_index].GetAcceleration(t);
        }

        inline Eigen::Vector3d GetJerk(double t) const
        {
            int piece_index = LocatePieceIndex(t);
            return pieces_[piece_index].GetJerk(t);
        }

        inline Eigen::Vector3d GetJunctionPosition(int junction_index) const
        {
            if (junction_index != GetPieceCount())
            {
                return pieces_[junction_index].GetCoefficientMatrix().col(5);
            }
            else
            {
                return pieces_[junction_index - 1].GetPosition(pieces_[junction_index - 1].GetDuration());
            }
        }

        inline Eigen::Vector3d GetJunctionVelocity(int junction_index) const
        {
            if (junction_index != GetPieceCount())
            {
                return pieces_[junction_index].GetCoefficientMatrix().col(4);
            }
            else
            {
                return pieces_[junction_index - 1].GetVelocity(pieces_[junction_index - 1].GetDuration());
            }
        }

        inline Eigen::Vector3d GetJunctionAcceleration(int junction_index) const
        {
            if (junction_index != GetPieceCount())
            {
                return pieces_[junction_index].GetCoefficientMatrix().col(3) * 2.0;
            }
            else
            {
                return pieces_[junction_index - 1].GetAcceleration(pieces_[junction_index - 1].GetDuration());
            }
        }

        inline double GetMaxVelocityRate() const
        {
            int piece_count = GetPieceCount();
            double max_velocity_rate = -INFINITY;
            double temporary_norm;
            for (int i = 0; i < piece_count; i++)
            {
                temporary_norm = pieces_[i].GetMaxVelocityRate();
                max_velocity_rate = max_velocity_rate < temporary_norm ? temporary_norm : max_velocity_rate;
            }
            return max_velocity_rate;
        }

        inline double GetMaxAccelerationRate() const
        {
            int piece_count = GetPieceCount();
            double max_acceleration_rate = -INFINITY;
            double temporary_norm;
            for (int i = 0; i < piece_count; i++)
            {
                temporary_norm = pieces_[i].GetMaxAccelerationRate();
                max_acceleration_rate = max_acceleration_rate < temporary_norm ? temporary_norm : max_acceleration_rate;
            }
            return max_acceleration_rate;
        }

        inline bool CheckMaxVelocityRate(const double &max_velocity_rate) const
        {
            int piece_count = GetPieceCount();
            bool feasible = true;
            for (int i = 0; i < piece_count && feasible; i++)
            {
                feasible = feasible && pieces_[i].CheckMaxVelocityRate(max_velocity_rate);
            }
            return feasible;
        }

        inline bool CheckMaxAccelerationRate(const double &max_acceleration_rate) const
        {
            int piece_count = GetPieceCount();
            bool feasible = true;
            for (int i = 0; i < piece_count && feasible; i++)
            {
                feasible = feasible && pieces_[i].CheckMaxAccelerationRate(max_acceleration_rate);
            }
            return feasible;
        }

        // GaaiLam
        inline Piece GetPiece(int i) const
        {
            return pieces_[i];
        }

        inline bool ProjectPoint(const Eigen::Vector3d &pt,
                               int &piece_index, double &piece_time, Eigen::Vector3d &projected_point)
        {
            bool found_projected_point = false;
            for (int i = 0; i < GetPieceCount(); ++i)
            {
                auto piece = pieces_[i];
                if (piece.ProjectPoint(pt, piece_time, projected_point))
                {
                    piece_index = i;
                    found_projected_point = true;
                    break;
                }
            }
            if (!found_projected_point)
            {
                // std::cout << "\033[32m" << "cannot project pt to traj" << "\033[0m" << std::endl;
                // std::cout << "pt: " << pt.transpose() << std::endl;
                // assert(false);
            }
            return found_projected_point;
        }
        inline bool IntersectPlane(const Eigen::Vector3d p,
                                       const Eigen::Vector3d v,
                                       int &piece_index, double &piece_time, Eigen::Vector3d &pt)
        {
            for (int i = 0; i < GetPieceCount(); ++i)
            {
                const auto &piece = pieces_[i];
                if (piece.IntersectPlane(p, v, piece_time, pt))
                {
                    piece_index = i;
                    return true;
                }
            }
            return false;
        }

        inline std::vector<Eigen::Vector3d> GetWaypoints()
        {
            std::vector<Eigen::Vector3d> pts;
            for (int i = 0; i < GetPieceCount(); ++i)
            {
                pts.push_back(pieces_[i].GetPosition(0));
            }
            return pts;
        }

        // zxzx
        inline std::pair<int, double> LocatePieceIndexWithRatio(double &t) const
        {
            int piece_count = GetPieceCount();
            int index;
            double duration;
            for (index = 0;
                 index < piece_count &&
                 t > (duration = pieces_[index].GetDuration());
                 index++)
            {
                t -= duration;
            }
            if (index == piece_count)
            {
                index--;
                t += pieces_[index].GetDuration();
            }
            std::pair<int, double> index_ratio;
            index_ratio.first = index;
            index_ratio.second = t / duration;
            return index_ratio;
        }

        inline Eigen::Vector3d GetPositionWithIndexRatio(double t, std::pair<int, double> &index_ratio) const
        {
            index_ratio = LocatePieceIndexWithRatio(t);
            return pieces_[index_ratio.first].GetPosition(t);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower and upper bandwidths.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void Create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            Destroy();
            size_ = n;
            lower_bandwidth_ = p;
            upper_bandwidth_ = q;
            int actual_size = size_ * (lower_bandwidth_ + upper_bandwidth_ + 1);
            data_ = new double[actual_size];
            std::fill_n(data_, actual_size, 0.0);
            return;
        }

        inline void Destroy()
        {
            if (data_ != nullptr)
            {
                delete[] data_;
                data_ = nullptr;
            }
            return;
        }

        inline void operator=(const BandedSystem &other)
        {
            data_ = nullptr;
            Create(other.size_, other.lower_bandwidth_, other.upper_bandwidth_);
            memcpy(data_, other.data_, size_ * (lower_bandwidth_ + upper_bandwidth_ + 1) * sizeof(double));
        }

    private:
        int size_;
        int lower_bandwidth_;
        int upper_bandwidth_;
        double *data_ = nullptr;

    public:
        // Reset the matrix to zero
        inline void Reset(void)
        {
            std::fill_n(data_, size_ * (lower_bandwidth_ + upper_bandwidth_ + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return data_[(i - j + upper_bandwidth_) * size_ + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return data_[(i - j + upper_bandwidth_) * size_ + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void FactorizeLu()
        {
            int max_row, max_column;
            double matrix_value;
            for (int k = 0; k <= size_ - 2; k++)
            {
                max_row = std::min(k + lower_bandwidth_, size_ - 1);
                matrix_value = operator()(k, k);
                for (int i = k + 1; i <= max_row; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= matrix_value;
                    }
                }
                max_column = std::min(k + upper_bandwidth_, size_ - 1);
                for (int j = k + 1; j <= max_column; j++)
                {
                    matrix_value = operator()(k, j);
                    if (matrix_value != 0.0)
                    {
                        for (int i = k + 1; i <= max_row; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * matrix_value;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be size_*m, i.e.,
        // m vectors to be solved.
        inline void Solve(Eigen::MatrixXd &b) const
        {
            int max_row;
            for (int j = 0; j <= size_ - 1; j++)
            {
                max_row = std::min(j + lower_bandwidth_, size_ - 1);
                for (int i = j + 1; i <= max_row; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = size_ - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                max_row = std::max(0, j - upper_bandwidth_);
                for (int i = max_row; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be size_*m, i.e.,
        // m vectors to be solved.
        inline void SolveAdjoint(Eigen::MatrixXd &b) const
        {
            int max_row;
            for (int j = 0; j <= size_ - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                max_row = std::min(j + upper_bandwidth_, size_ - 1);
                for (int i = j + 1; i <= max_row; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = size_ - 1; j >= 0; j--)
            {
                max_row = std::max(0, j - lower_bandwidth_);
                for (int i = max_row; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    class MinJerkOpt
    {
    public:
        inline void operator=(const MinJerkOpt &other)
        {
            piece_count_ = other.piece_count_;
            head_state_ = other.head_state_;
            tail_state_ = other.tail_state_;
            durations_ = other.durations_;
            banded_system_ = other.banded_system_;
            coefficients_ = other.coefficients_;
            durations_squared_ = other.durations_squared_;
            durations_cubed_ = other.durations_cubed_;
            durations_fourth_ = other.durations_fourth_;
            durations_fifth_ = other.durations_fifth_;
            coefficient_gradients_ = other.coefficient_gradients_;
        }
        ~MinJerkOpt() { banded_system_.Destroy(); }

    private:
        int piece_count_;
        Eigen::Matrix3d head_state_;
        Eigen::Matrix3d tail_state_;
        Eigen::VectorXd durations_;
        BandedSystem banded_system_;
        Eigen::MatrixXd coefficients_;

        // Temp variables
        Eigen::VectorXd durations_squared_;
        Eigen::VectorXd durations_cubed_;
        Eigen::VectorXd durations_fourth_;
        Eigen::VectorXd durations_fifth_;
        Eigen::MatrixXd coefficient_gradients_;

    private:
        template <typename EigenVectorType>
        inline void AddJerkGradientToDurations(EigenVectorType &duration_gradients) const
        {
            for (int i = 0; i < piece_count_; i++)
            {
                duration_gradients(i) += 36.0 * coefficients_.row(6 * i + 3).squaredNorm() +
                          288.0 * coefficients_.row(6 * i + 4).dot(coefficients_.row(6 * i + 3)) * durations_(i) +
                          576.0 * coefficients_.row(6 * i + 4).squaredNorm() * durations_squared_(i) +
                          720.0 * coefficients_.row(6 * i + 5).dot(coefficients_.row(6 * i + 3)) * durations_squared_(i) +
                          2880.0 * coefficients_.row(6 * i + 5).dot(coefficients_.row(6 * i + 4)) * durations_cubed_(i) +
                          3600.0 * coefficients_.row(6 * i + 5).squaredNorm() * durations_fourth_(i);
            }
            return;
        }

        template <typename EigenMatrixType>
        inline void AddJerkGradientToCoefficients(EigenMatrixType &coefficient_gradients) const
        {
            for (int i = 0; i < piece_count_; i++)
            {
                coefficient_gradients.row(6 * i + 5) += 240.0 * coefficients_.row(6 * i + 3) * durations_cubed_(i) +
                                      720.0 * coefficients_.row(6 * i + 4) * durations_fourth_(i) +
                                      1440.0 * coefficients_.row(6 * i + 5) * durations_fifth_(i);
                coefficient_gradients.row(6 * i + 4) += 144.0 * coefficients_.row(6 * i + 3) * durations_squared_(i) +
                                      384.0 * coefficients_.row(6 * i + 4) * durations_cubed_(i) +
                                      720.0 * coefficients_.row(6 * i + 5) * durations_fourth_(i);
                coefficient_gradients.row(6 * i + 3) += 72.0 * coefficients_.row(6 * i + 3) * durations_(i) +
                                      144.0 * coefficients_.row(6 * i + 4) * durations_squared_(i) +
                                      240.0 * coefficients_.row(6 * i + 5) * durations_cubed_(i);
            }
            return;
        }

        inline void SolveAdjointCoefficientGradient(Eigen::MatrixXd &coefficient_gradients) const
        {
            banded_system_.SolveAdjoint(coefficient_gradients);
            return;
        }

        template <typename EigenVectorType>
        inline void AddCoefficientGradientToDurations(const Eigen::MatrixXd &adjoint_coefficient_gradients, EigenVectorType &duration_gradients) const
        {
            Eigen::MatrixXd first_block(6, 3), second_block(3, 3);

            Eigen::RowVector3d negative_velocity, negative_acceleration, negative_jerk, negative_snap, negative_crackle;

            for (int i = 0; i < piece_count_ - 1; i++)
            {
                negative_velocity = -(coefficients_.row(i * 6 + 1) +
                           2.0 * durations_(i) * coefficients_.row(i * 6 + 2) +
                           3.0 * durations_squared_(i) * coefficients_.row(i * 6 + 3) +
                           4.0 * durations_cubed_(i) * coefficients_.row(i * 6 + 4) +
                           5.0 * durations_fourth_(i) * coefficients_.row(i * 6 + 5));
                negative_acceleration = -(2.0 * coefficients_.row(i * 6 + 2) +
                           6.0 * durations_(i) * coefficients_.row(i * 6 + 3) +
                           12.0 * durations_squared_(i) * coefficients_.row(i * 6 + 4) +
                           20.0 * durations_cubed_(i) * coefficients_.row(i * 6 + 5));
                negative_jerk = -(6.0 * coefficients_.row(i * 6 + 3) +
                           24.0 * durations_(i) * coefficients_.row(i * 6 + 4) +
                           60.0 * durations_squared_(i) * coefficients_.row(i * 6 + 5));
                negative_snap = -(24.0 * coefficients_.row(i * 6 + 4) +
                           120.0 * durations_(i) * coefficients_.row(i * 6 + 5));
                negative_crackle = -120.0 * coefficients_.row(i * 6 + 5);

                first_block << negative_snap, negative_crackle, negative_velocity, negative_velocity, negative_acceleration, negative_jerk;

                duration_gradients(i) += first_block.cwiseProduct(adjoint_coefficient_gradients.block<6, 3>(6 * i + 3, 0)).sum();
            }

            negative_velocity = -(coefficients_.row(6 * piece_count_ - 5) +
                       2.0 * durations_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 4) +
                       3.0 * durations_squared_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 3) +
                       4.0 * durations_cubed_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 2) +
                       5.0 * durations_fourth_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 1));
            negative_acceleration = -(2.0 * coefficients_.row(6 * piece_count_ - 4) +
                       6.0 * durations_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 3) +
                       12.0 * durations_squared_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 2) +
                       20.0 * durations_cubed_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 1));
            negative_jerk = -(6.0 * coefficients_.row(6 * piece_count_ - 3) +
                       24.0 * durations_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 2) +
                       60.0 * durations_squared_(piece_count_ - 1) * coefficients_.row(6 * piece_count_ - 1));

            second_block << negative_velocity, negative_acceleration, negative_jerk;

            duration_gradients(piece_count_ - 1) += second_block.cwiseProduct(adjoint_coefficient_gradients.block<3, 3>(6 * piece_count_ - 3, 0)).sum();

            return;
        }

        template <typename EigenMatrixType>
        inline void AddCoefficientGradientToInnerPoints(const Eigen::MatrixXd &adjoint_coefficient_gradients, EigenMatrixType &inner_point_gradients) const
        {
            for (int i = 0; i < piece_count_ - 1; i++)
            {
                // inner_point_gradients.col(i) += adjoint_coefficient_gradients.row(6 * i + 5).transpose();
                inner_point_gradients.col(i) = adjoint_coefficient_gradients.row(6 * i + 5).transpose(); // zxzx
            }
            return;
        }

        template <typename EigenVectorType>
        inline void AddTimeIntegralPenalty(const Eigen::VectorXi constraint_counts,
                                      const Eigen::VectorXi &corridor_indices,
                                      const std::vector<Eigen::MatrixXd> &corridor_configurations,
                                      const double max_velocity,
                                      const double max_acceleration,
                                      const Eigen::Vector3d penalty_weights,
                                      double &cost,
                                      EigenVectorType &duration_gradients,
                                      Eigen::MatrixXd &coefficient_gradients) const
        {
            double penalty = 0.0;
            const double max_velocity_squared = max_velocity * max_velocity;
            const double max_acceleration_squared = max_acceleration * max_acceleration;

            Eigen::Vector3d position, velocity, acceleration, jerk;
            double step, alpha;
            double time, time_squared, time_cubed, time_fourth, time_fifth;
            Eigen::Matrix<double, 6, 1> position_basis, velocity_basis, acceleration_basis, jerk_basis;
            Eigen::Vector3d outer_normal;
            int plane_count;
            double position_violation, velocity_violation, acceleration_violation;
            double position_penalty_derivative, velocity_penalty_derivative, acceleration_penalty_derivative;
            double position_penalty, velocity_penalty, acceleration_penalty;
            Eigen::Matrix<double, 6, 3> velocity_coefficient_gradient, acceleration_coefficient_gradient;
            double velocity_duration_gradient, acceleration_duration_gradient;
            double integration_weight;

            int integration_point_count, corridor_index;
            for (int i = 0; i < piece_count_; i++)
            {
                const auto &c = coefficients_.block<6, 3>(i * 6, 0);
                step = durations_(i) / constraint_counts(i);
                time = 0.0;
                integration_point_count = constraint_counts(i) + 1;
                for (int j = 0; j < integration_point_count; j++)
                {
                    time_squared = time * time;
                    time_cubed = time_squared * time;
                    time_fourth = time_squared * time_squared;
                    time_fifth = time_fourth * time;
                    position_basis << 1.0, time, time_squared, time_cubed, time_fourth, time_fifth;
                    velocity_basis << 0.0, 1.0, 2.0 * time, 3.0 * time_squared, 4.0 * time_cubed, 5.0 * time_fourth;
                    acceleration_basis << 0.0, 0.0, 2.0, 6.0 * time, 12.0 * time_squared, 20.0 * time_cubed;
                    jerk_basis << 0.0, 0.0, 0.0, 6.0, 24.0 * time, 60.0 * time_squared;
                    alpha = 1.0 / constraint_counts(i) * j;
                    position = c.transpose() * position_basis;
                    velocity = c.transpose() * velocity_basis;
                    acceleration = c.transpose() * acceleration_basis;
                    jerk = c.transpose() * jerk_basis;
                    velocity_violation = velocity.squaredNorm() - max_velocity_squared;
                    acceleration_violation = acceleration.squaredNorm() - max_acceleration_squared;

                    integration_weight = (j == 0 || j == integration_point_count - 1) ? 0.5 : 1.0;

                    corridor_index = corridor_indices(i);
                    plane_count = corridor_configurations[corridor_index].cols();
                    for (int k = 0; k < plane_count; k++)
                    {
                        outer_normal = corridor_configurations[corridor_index].col(k).head<3>();
                        position_violation = outer_normal.dot(position - corridor_configurations[corridor_index].col(k).tail<3>());
                        if (position_violation > 0.0)
                        {
                            position_penalty_derivative = position_violation * position_violation;
                            position_penalty = position_penalty_derivative * position_violation;
                            position_penalty_derivative *= 3.0;
                            coefficient_gradients.block<6, 3>(i * 6, 0) += integration_weight * step * penalty_weights(0) * position_penalty_derivative * position_basis * outer_normal.transpose();
                            duration_gradients(i) += integration_weight * (penalty_weights(0) * position_penalty_derivative * alpha * outer_normal.dot(velocity) * step +
                                             penalty_weights(0) * position_penalty / constraint_counts(i));
                            penalty += integration_weight * step * penalty_weights(0) * position_penalty;
                        }
                    }

                    if (velocity_violation > 0.0)
                    {
                        velocity_penalty_derivative = velocity_violation * velocity_violation;
                        velocity_penalty = velocity_penalty_derivative * velocity_violation;
                        velocity_penalty_derivative *= 3.0;
                        velocity_coefficient_gradient = 2.0 * velocity_basis * velocity.transpose();
                        velocity_duration_gradient = 2.0 * alpha * velocity.transpose() * acceleration;
                        coefficient_gradients.block<6, 3>(i * 6, 0) += integration_weight * step * penalty_weights(1) * velocity_penalty_derivative * velocity_coefficient_gradient;
                        duration_gradients(i) += integration_weight * (penalty_weights(1) * velocity_penalty_derivative * velocity_duration_gradient * step +
                                         penalty_weights(1) * velocity_penalty / constraint_counts(i));
                        penalty += integration_weight * step * penalty_weights(1) * velocity_penalty;
                    }

                    if (acceleration_violation > 0.0)
                    {
                        acceleration_penalty_derivative = acceleration_violation * acceleration_violation;
                        acceleration_penalty = acceleration_penalty_derivative * acceleration_violation;
                        acceleration_penalty_derivative *= 3.0;
                        acceleration_coefficient_gradient = 2.0 * acceleration_basis * acceleration.transpose();
                        acceleration_duration_gradient = 2.0 * alpha * acceleration.transpose() * jerk;
                        coefficient_gradients.block<6, 3>(i * 6, 0) += integration_weight * step * penalty_weights(2) * acceleration_penalty_derivative * acceleration_coefficient_gradient;
                        duration_gradients(i) += integration_weight * (penalty_weights(2) * acceleration_penalty_derivative * acceleration_duration_gradient * step +
                                         penalty_weights(2) * acceleration_penalty / constraint_counts(i));
                        penalty += integration_weight * step * penalty_weights(2) * acceleration_penalty;
                    }

                    time += step;
                }
            }

            cost += penalty;
            return;
        }

    public:
        inline void Reset(const Eigen::Matrix3d &head_state,
                          const Eigen::Matrix3d &tail_state,
                          const int &piece_count)
        {
            piece_count_ = piece_count;
            head_state_ = head_state;
            tail_state_ = tail_state;
            durations_.resize(piece_count_);
            banded_system_.Create(6 * piece_count_, 6, 6);
            coefficients_.resize(6 * piece_count_, 3);
            coefficient_gradients_.resize(6 * piece_count_, 3);
            // duration_gradients.resize(6 * piece_count_);
            return;
        }

        inline void Generate(const Eigen::MatrixXd &inner_points,
                             const Eigen::VectorXd &durations)
        {
            if (inner_points.cols() == 0)
            {

                durations_(0) = durations(0);
                double t1_inv = 1.0 / durations_(0);
                double t2_inv = t1_inv * t1_inv;
                double t3_inv = t2_inv * t1_inv;
                double t4_inv = t2_inv * t2_inv;
                double t5_inv = t4_inv * t1_inv;
                CoefficientMatrix reversed_coefficients;
                reversed_coefficients.col(5) = 0.5 * (tail_state_.col(2) - head_state_.col(2)) * t3_inv -
                                          3.0 * (head_state_.col(1) + tail_state_.col(1)) * t4_inv +
                                          6.0 * (tail_state_.col(0) - head_state_.col(0)) * t5_inv;
                reversed_coefficients.col(4) = (-tail_state_.col(2) + 1.5 * head_state_.col(2)) * t2_inv +
                                          (8.0 * head_state_.col(1) + 7.0 * tail_state_.col(1)) * t3_inv +
                                          15.0 * (-tail_state_.col(0) + head_state_.col(0)) * t4_inv;
                reversed_coefficients.col(3) = (0.5 * tail_state_.col(2) - 1.5 * head_state_.col(2)) * t1_inv -
                                          (6.0 * head_state_.col(1) + 4.0 * tail_state_.col(1)) * t2_inv +
                                          10.0 * (tail_state_.col(0) - head_state_.col(0)) * t3_inv;
                reversed_coefficients.col(2) = 0.5 * head_state_.col(2);
                reversed_coefficients.col(1) = head_state_.col(1);
                reversed_coefficients.col(0) = head_state_.col(0);
                coefficients_ = reversed_coefficients.transpose();
            }
            else
            {
                durations_ = durations;
                durations_squared_ = durations_.cwiseProduct(durations_);
                durations_cubed_ = durations_squared_.cwiseProduct(durations_);
                durations_fourth_ = durations_squared_.cwiseProduct(durations_squared_);
                durations_fifth_ = durations_fourth_.cwiseProduct(durations_);

                banded_system_.Reset();
                coefficients_.setZero();

                banded_system_(0, 0) = 1.0;
                banded_system_(1, 1) = 1.0;
                banded_system_(2, 2) = 2.0;
                coefficients_.row(0) = head_state_.col(0).transpose();
                coefficients_.row(1) = head_state_.col(1).transpose();
                coefficients_.row(2) = head_state_.col(2).transpose();

                for (int i = 0; i < piece_count_ - 1; i++)
                {
                    banded_system_(6 * i + 3, 6 * i + 3) = 6.0;
                    banded_system_(6 * i + 3, 6 * i + 4) = 24.0 * durations_(i);
                    banded_system_(6 * i + 3, 6 * i + 5) = 60.0 * durations_squared_(i);
                    banded_system_(6 * i + 3, 6 * i + 9) = -6.0;
                    banded_system_(6 * i + 4, 6 * i + 4) = 24.0;
                    banded_system_(6 * i + 4, 6 * i + 5) = 120.0 * durations_(i);
                    banded_system_(6 * i + 4, 6 * i + 10) = -24.0;
                    banded_system_(6 * i + 5, 6 * i) = 1.0;
                    banded_system_(6 * i + 5, 6 * i + 1) = durations_(i);
                    banded_system_(6 * i + 5, 6 * i + 2) = durations_squared_(i);
                    banded_system_(6 * i + 5, 6 * i + 3) = durations_cubed_(i);
                    banded_system_(6 * i + 5, 6 * i + 4) = durations_fourth_(i);
                    banded_system_(6 * i + 5, 6 * i + 5) = durations_fifth_(i);
                    banded_system_(6 * i + 6, 6 * i) = 1.0;
                    banded_system_(6 * i + 6, 6 * i + 1) = durations_(i);
                    banded_system_(6 * i + 6, 6 * i + 2) = durations_squared_(i);
                    banded_system_(6 * i + 6, 6 * i + 3) = durations_cubed_(i);
                    banded_system_(6 * i + 6, 6 * i + 4) = durations_fourth_(i);
                    banded_system_(6 * i + 6, 6 * i + 5) = durations_fifth_(i);
                    banded_system_(6 * i + 6, 6 * i + 6) = -1.0;
                    banded_system_(6 * i + 7, 6 * i + 1) = 1.0;
                    banded_system_(6 * i + 7, 6 * i + 2) = 2 * durations_(i);
                    banded_system_(6 * i + 7, 6 * i + 3) = 3 * durations_squared_(i);
                    banded_system_(6 * i + 7, 6 * i + 4) = 4 * durations_cubed_(i);
                    banded_system_(6 * i + 7, 6 * i + 5) = 5 * durations_fourth_(i);
                    banded_system_(6 * i + 7, 6 * i + 7) = -1.0;
                    banded_system_(6 * i + 8, 6 * i + 2) = 2.0;
                    banded_system_(6 * i + 8, 6 * i + 3) = 6 * durations_(i);
                    banded_system_(6 * i + 8, 6 * i + 4) = 12 * durations_squared_(i);
                    banded_system_(6 * i + 8, 6 * i + 5) = 20 * durations_cubed_(i);
                    banded_system_(6 * i + 8, 6 * i + 8) = -2.0;

                    coefficients_.row(6 * i + 5) = inner_points.col(i).transpose();
                }

                banded_system_(6 * piece_count_ - 3, 6 * piece_count_ - 6) = 1.0;
                banded_system_(6 * piece_count_ - 3, 6 * piece_count_ - 5) = durations_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 3, 6 * piece_count_ - 4) = durations_squared_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 3, 6 * piece_count_ - 3) = durations_cubed_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 3, 6 * piece_count_ - 2) = durations_fourth_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 3, 6 * piece_count_ - 1) = durations_fifth_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 2, 6 * piece_count_ - 5) = 1.0;
                banded_system_(6 * piece_count_ - 2, 6 * piece_count_ - 4) = 2 * durations_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 2, 6 * piece_count_ - 3) = 3 * durations_squared_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 2, 6 * piece_count_ - 2) = 4 * durations_cubed_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 2, 6 * piece_count_ - 1) = 5 * durations_fourth_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 1, 6 * piece_count_ - 4) = 2;
                banded_system_(6 * piece_count_ - 1, 6 * piece_count_ - 3) = 6 * durations_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 1, 6 * piece_count_ - 2) = 12 * durations_squared_(piece_count_ - 1);
                banded_system_(6 * piece_count_ - 1, 6 * piece_count_ - 1) = 20 * durations_cubed_(piece_count_ - 1);

                coefficients_.row(6 * piece_count_ - 3) = tail_state_.col(0).transpose();
                coefficients_.row(6 * piece_count_ - 2) = tail_state_.col(1).transpose();
                coefficients_.row(6 * piece_count_ - 1) = tail_state_.col(2).transpose();

                banded_system_.FactorizeLu();
                banded_system_.Solve(coefficients_);

                return;
            }
        }

        inline const Eigen::MatrixXd &GetCoefficients() const
        {
            return coefficients_;
        }

        inline const Eigen::VectorXd &GetDurations() const
        {
            return durations_;
        }

        inline Eigen::MatrixXd &GetCoefficientGradients()
        {
            return coefficient_gradients_;
        }

        // inline Eigen::MatrixXd GetDurationGradients() const
        // {
        //     return duration_gradients;
        // }

        // inline Eigen::MatrixXd GetDurationGradient(size_t i) const
        // {
        //     return duration_gradients(i);
        // }

        inline double GetTrajectoryJerkCost() const
        {
            double objective = 0.0;
            for (int i = 0; i < piece_count_; i++)
            {
                objective += 36.0 * coefficients_.row(6 * i + 3).squaredNorm() * durations_(i) +
                             144.0 * coefficients_.row(6 * i + 4).dot(coefficients_.row(6 * i + 3)) * durations_squared_(i) +
                             192.0 * coefficients_.row(6 * i + 4).squaredNorm() * durations_cubed_(i) +
                             240.0 * coefficients_.row(6 * i + 5).dot(coefficients_.row(6 * i + 3)) * durations_cubed_(i) +
                             720.0 * coefficients_.row(6 * i + 5).dot(coefficients_.row(6 * i + 4)) * durations_fourth_(i) +
                             720.0 * coefficients_.row(6 * i + 5).squaredNorm() * durations_fifth_(i);
            }
            return objective;
        }

        inline Trajectory GetTrajectory(void) const
        {
            Trajectory traj;
            traj.Reserve(piece_count_);
            for (int i = 0; i < piece_count_; i++)
            {
                traj.EmplaceBack(durations_(i), coefficients_.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
            return traj;
        }

        inline Eigen::MatrixXd GetInitialConstraintPoints(const int samples_per_piece) const
        {
            Eigen::MatrixXd pts(3, piece_count_ * samples_per_piece + 1);
            Eigen::Vector3d position;
            Eigen::Matrix<double, 6, 1> position_basis;
            double time, time_squared, time_cubed, time_fourth, time_fifth;
            double step;
            int point_index = 0;

            for (int i = 0; i < piece_count_; ++i)
            {
                const auto &c = coefficients_.block<6, 3>(i * 6, 0);
                step = durations_(i) / samples_per_piece;
                time = 0.0;
                double t = 0;
                // integration_point_count = samples_per_piece;

                for (int j = 0; j <= samples_per_piece; ++j)
                {
                    time_squared = time * time;
                    time_cubed = time_squared * time;
                    time_fourth = time_squared * time_squared;
                    time_fifth = time_fourth * time;
                    position_basis << 1.0, time, time_squared, time_cubed, time_fourth, time_fifth;
                    position = c.transpose() * position_basis;
                    pts.col(point_index) = position;

                    time += step;
                    if (j != samples_per_piece || (j == samples_per_piece && i == piece_count_ - 1))
                    {
                        ++point_index;
                    }
                }
            }

            return pts;
        }

        template <typename EigenVectorType, typename EigenMatrixType>
        inline void GetGradientsToTimeAndPoints(EigenVectorType &duration_gradients,
                               EigenMatrixType &inner_point_gradients)
        {
            SolveAdjointCoefficientGradient(coefficient_gradients_);
            AddCoefficientGradientToDurations(coefficient_gradients_, duration_gradients);
            AddCoefficientGradientToInnerPoints(coefficient_gradients_, inner_point_gradients);
        }

        template <typename EigenVectorType>
        inline void InitializeGradientCost(EigenVectorType &duration_gradients,
                                 double &cost)
        {
            // printf( "inner_point_gradients=%d\n", inner_point_gradients.size() );

            duration_gradients.setZero();
            coefficient_gradients_.setZero();
            cost = GetTrajectoryJerkCost();
            AddJerkGradientToDurations(duration_gradients);
            AddJerkGradientToCoefficients(coefficient_gradients_);
        }

        template <typename EigenVectorType, typename EigenMatrixType>
        inline void EvaluateTrajectoryCostGradient(const Eigen::VectorXi &constraint_counts,
                                     const Eigen::VectorXi &corridor_indices,
                                     const std::vector<Eigen::MatrixXd> &corridor_configurations,
                                     const double &max_velocity,
                                     const double &max_acceleration,
                                     const Eigen::Vector3d &penalty_weights,
                                     double &cost,
                                     EigenVectorType &duration_gradients,
                                     EigenMatrixType &inner_point_gradients)
        {
            duration_gradients.setZero();
            inner_point_gradients.setZero();
            coefficient_gradients_.setZero();

            cost = GetTrajectoryJerkCost();
            AddJerkGradientToDurations(duration_gradients);
            AddJerkGradientToCoefficients(coefficient_gradients_);

            AddTimeIntegralPenalty(constraint_counts, corridor_indices, corridor_configurations,
                                   max_velocity, max_acceleration, penalty_weights, cost,
                                   duration_gradients, coefficient_gradients_);

            SolveAdjointCoefficientGradient(coefficient_gradients_);
            AddCoefficientGradientToDurations(coefficient_gradients_, duration_gradients);
            AddCoefficientGradientToInnerPoints(coefficient_gradients_, inner_point_gradients);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

}  // namespace poly_traj
