#ifndef ROBOT_CONTROLLER_MATRIX_HPP
#define ROBOT_CONTROLLER_MATRIX_HPP

#include <array>
#include <cmath>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <type_traits>

namespace robot_controller
{
/**
 * \brief Create a matrix with numerical values.
 *
 * \tparam T The numerical type to elements have.
 * \tparam m Amount of rows
 * \tparam n Amount of columns
 */
template<typename T, std::size_t m, std::size_t n> class Matrix
{
	static_assert(std::is_arithmetic<T>::value,
		"Matrix can only be declared with a type where "
		"std::is_arithmetic is true.");

public:
	/**
	 * \brief Get identity matrix
	 *
	 * \return Identity matrix.
	 */
	static constexpr Matrix<T, m, n> get_identity() noexcept;

	/**
	 * \brief Create matrix with all elements set to 0.
	 *
	 */
	Matrix() = default;

	/**
	 * \brief Default destructor.
	 *
	 */
	~Matrix() = default;

	/**
	 * \brief Default copy constructor.
	 *
	 * \param mat The matrix to copy.
	 */
	Matrix(const Matrix<T, m, n> &mat) = default;

	/**
	 * \brief Default move constructor.
	 *
	 * \param mat The matrix to move.
	 */
	Matrix(Matrix<T, m, n> &&mat) = default;

	/**
	 * \brief Default assignment operator.
	 *
	 * \param mat The matrix that is assigned.
	 * \return Reference to a matrix equivalent to mat.
	 */
	Matrix<T, m, n> &operator=(const Matrix<T, m, n> &mat) = default;

	/**
	 * \brief Default move assignment operator.
	 *
	 * \param mat The rvalue matrix that is assigned.
	 * \return Reference to a matrix equivalent to mat.
	 */
	Matrix<T, m, n> &operator=(Matrix<T, m, n> &&mat) = default;

	/**
	 * \brief Create matrix. Throws invalid argument if passed data doesn't
	 * match with amount of rows or columns.
	 *
	 * \param matrix_data matrix_data The data to initialize matrix with.
	 */
	constexpr Matrix(
		std::initializer_list<std::initializer_list<T>> matrix_data);

	/**
	 * \brief Get the amount of rows of the matrix.
	 *
	 * \return amount of rows.
	 */
	constexpr std::size_t get_m() const noexcept;

	/**
	 * \brief Get the amount of cols of the matrix.
	 *
	 * \return amount of cols.
	 */
	constexpr std::size_t get_n() const noexcept;

	/**
	 * \brief Get row of matrix at pos. No bounds checkin.
	 *
	 * \param pos pos The row to fetch.
	 * \return array containing the requested row.
	 */
	constexpr std::array<T, n> &operator[](std::size_t pos) noexcept;

	/**
	 * \brief Get row of matrix at pos. No bounds checkin.
	 *
	 * \param pos The row to fetch.
	 * \return array containing the requested row.
	 */
	constexpr const std::array<T, n> &operator[](std::size_t pos) const
		noexcept;

	/**
	 * \brief Get row of matrix at pos. Throw out of range exception if out
	 * of range.
	 *
	 * \param pos The row to fetch.
	 * \return array containing the requested row.
	 */
	constexpr std::array<T, n> &at(std::size_t pos);

	/**
	 * \brief Get row of matrix at pos. Throw out of range exception if out
	 * of range.
	 *
	 * \param pos The row to fetch.
	 * \return array containing the requested row.
	 */
	constexpr const std::array<T, n> &at(std::size_t pos) const;

	/**
	 * \brief Fuzzy compare 2 matrices.
	 *
	 * \param mat The matrix to compare with,
	 * \param fuzz The maximum allowed difference between each value where
	 * they are still considered fuzzy equal. \return true When fuzzy equal
	 * \return false When not equal
	 */
	constexpr bool fuzzy_equal(const Matrix<T, m, n> &mat, T fuzz) const
		noexcept;

	/**
	 * \brief Compare two matrices.
	 *
	 * \param mat The matrix to compare this one with.
	 * \return true If equal.
	 * \return false If not equal.
	 */
	constexpr bool operator==(const Matrix<T, m, n> &mat) const noexcept;

	/**
	 * \brief Compare two matrices.
	 *
	 * \param mat The matrix to compare this one with.
	 * \return true If equal.
	 * \return false If not equal.
	 */
	constexpr bool operator!=(const Matrix<T, m, n> &mat) const noexcept;

	/**
	 * \brief Add a scalar from every element of the matrix.
	 *
	 * \param scalar The scalar to add.

	 */
	constexpr void operator+=(T scalar) noexcept;

	/**
	 * \brief Subtract a scalar from every element of the matrix.
	 *
	 * \param scalar The scalar to subtract.
	 */
	constexpr void operator-=(T scalar) noexcept;

	/**
	 * \brief Multiply every element of the matrix with a scalar.
	 *
	 * \param scalar scalar The scalar to multiply with.
	 */
	constexpr void operator*=(T scalar) noexcept;
	/**
	 * \brief Divide every element of the matrix with a scalar.
	 *
	 * \param scalar The scalar to divide with.
	 */
	constexpr void operator/=(T scalar) noexcept;

	/**
	 * \brief Add a matrix to another matrix.
	 *
	 * \param mat The matrix to add to the current one.
	 */
	constexpr void operator+=(const Matrix<T, m, n> &mat) noexcept;

	/**
	 * \brief Subtract a matrix to another matrix.
	 *
	 * \param mat the matrix to subtract from the current one.
	 */
	constexpr void operator-=(const Matrix<T, m, n> &mat) noexcept;

	/**
	 * \brief Add a matrix to another matrix. Returns a new matrix.
	 *
	 * \param mat The matrix to add to the current one.
	 * \return A new matrix containing the summed elements.
	 */
	constexpr Matrix<T, m, n> operator+(const Matrix<T, m, n> &mat) const
		noexcept;

	/**
	 * \brief Subtract a matrix from another matrix. Returns a new matrix.
	 *
	 * \param mat The matrix to subtract from the current one.
	 * \return A new matrix containing the subtracted elements.
	 */
	constexpr Matrix<T, m, n> operator-(const Matrix<T, m, n> &mat) const
		noexcept;

	/**
	 * \brief Multiply a matrix with the current matrix.
	 *
	 * \tparam p The columns of the matrix to multiply with.
	 * \param mat The matrix to multiply with.
	 * \return A new matrix that is the result of multiplying the current
	 * matrix with the provided one.
	 */
	template<std::size_t p>
	constexpr Matrix<T, m, p> operator*(const Matrix<T, n, p> &mat) const
		noexcept;

	/**
	 * \brief Right augment the current matrix with the provided matrix.
	 *
	 * \tparam p The columns of the matrix to augment with.
	 * \param mat The matrix that the current one will be augmented with.
	 * \return A new matrix that is the current one augmented with the
	 * provided one.
	 */
	template<std::size_t p>
	constexpr Matrix<T, m, n + p> horizontal_augment(
		const Matrix<T, m, p> &mat) const noexcept;

	/**
	 * \brief Bottom augment the current matrix with the provided matrix.
	 *
	 * \tparam p The rows of the matrix to augment with.
	 * \param mat The matrix that the current one will be augmented with.
	 * \return A new matrix that is the current one augmented with the
	 * provided one.
	 */
	template<std::size_t p>
	constexpr Matrix<T, m + p, n> vertical_augment(
		const Matrix<T, p, n> &mat) const noexcept;

	/**
	 * \brief Create a new matrix from the current one where some columns
	 * and rows are sliced of.
	 *
	 * \tparam y The amount of rows to slice of from the top.
	 * \tparam x The amount of columns to slice of from the bottom.
	 * \tparam p The amount of rows to take. If larger then the original
	 * matrix the elements will be filled with 0. \tparam q The amount of
	 * columns to take. If larger then the original matrix the elements will
	 * be filled with 0. \return A new matrix which is subset of the
	 * original.
	 */
	template<std::size_t y, std::size_t x, std::size_t p, std::size_t q>
	constexpr Matrix<T, p, q> slice() const noexcept;

	/**
	 * \brief Transpose the matrix.
	 *
	 * \return A new matrix that is is the transpose of the current one.
	 */
	constexpr Matrix<T, n, m> transpose() const noexcept;

	/**
	 * \brief Get the inverse of the matrix, implemented by using
	 * gauss-jordan. This operation is only available for square matrices/
	 *
	 * \return The inversed matrix if it is available.
	 */
	constexpr std::optional<Matrix<T, m, n>> inverse() const noexcept;

	/**
	 * \brief Get the magnitude of the matrix. This is equivalent to the
	 * euclidean distance.
	 *
	 * \return The calculated magnitude.
	 */
	constexpr T get_magnitude() const noexcept;

	/**
	 * \brief Get a string representation.
	 *
	 * \return The string representation.
	 */
	std::string str() const;

private:
	/**
	 * \brief Underlying array containg the matrix elements.
	 *
	 */
	std::array<std::array<T, n>, m> data{};
};

/**
 * \brief Add a scalar from every element of the matrix. Doesn't modify
 * original matrix.
 *
 * \param lhs The matrix to base the addition on.
 * \param rhs The scalar to add.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator+(const Matrix<T, m, n> &lhs, T rhs) noexcept;

/**
 * \brief Add a scalar from every element of the matrix. Doesn't modify
 * original matrix.
 *
 * \param lhs lhs The scalar to add.
 * \param rhs rhs The matrix to subtract the scalar from.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator+(T lhs, const Matrix<T, m, n> &rhs) noexcept;

/**
 * \brief Subtract a scalar from every element of the matrix. Doesn't modify
 * original matrix.
 *
 * \param lhs lhs The matrix to subtract the scalard from.
 * \param rhs rhs The scalar to subtract.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator-(const Matrix<T, m, n> &lhs, T rhs) noexcept;

/**
 * \brief Subtract a scalar from every element of the matrix. Doesn't modify
 * original matrix.
 *
 * \param lhs The matrix to subtract the scalard from.
 * \param rhs The scalar to subtract.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator-(T lhs, const Matrix<T, m, n> &rhs) noexcept;

/**
 * \brief Multiply every element of the matrix with a scalar. Doesn't modify
 * original matrix.
 *
 * \param lhs The matrix that will be used to multiplied.
 * \param rhs The scalar to multiply with.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator*(const Matrix<T, m, n> &lhs, T rhs) noexcept;

/**
 * \brief Multiply every element of the matrix with a scalar. Doesn't modify
 * original matrix.
 *
 * \param lhs lhs The scalar to multiply with.
 * \param rhs rhs The matrix that will be used to multiplied.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator*(T lhs, const Matrix<T, m, n> &rhs) noexcept;

/**
 * \brief Divide every element of the matrix with a scalar. Doesn't modify
 * original matrix.
 *
 * \param lhs The matrix that will be divided.
 * \param rhs The scalar to divide with.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator/(const Matrix<T, m, n> &lhs, T rhs) noexcept;

/**
 * \brief Divide every element of the matrix with a scalar. Doesn't modify
 * original matrix.
 *
 * \param lhs The scalar to divide with.
 * \param rhs The matrix that will be divided.
 * \return A new matrix with the modified elements.
 */
template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator/(T lhs, const Matrix<T, m, n> &rhs) noexcept;

/**
 * \brief Output the matrix to an ostream.
 *
 * \param os The ostream that.
 * \param mat The matrix that contains.
 * \return The ostream where the matrix has been outputted to.
 */
template<typename T, std::size_t m, std::size_t n>
std::ostream &operator<<(std::ostream &os, const Matrix<T, m, n> &mat);

#include "Matrix.ipp"
} // robot_controller

#endif