template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> Matrix<T, m, n>::get_identity() noexcept
{
	auto identity = Matrix<T, m, m>();

	for (std::size_t i = 0; i < m; ++i) {
		identity[i][i] = 1;
	}

	return identity;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n>::Matrix(
	const std::initializer_list<std::initializer_list<T>> matrix_data)
{
	if (matrix_data.size() != m) {
		throw std::invalid_argument("Invalid amount of rows.");
	}

	for (const auto &col : matrix_data) {
		if (col.size() != n) {
			throw std::invalid_argument(
				"Invalid amount of columns.");
		}
	}

	std::size_t pos_i = 0;
	std::size_t pos_j = 0;

	for (auto i = matrix_data.begin(); i != matrix_data.end(); ++i) {
		for (auto j = i->begin(); j != i->end(); ++j) {
			this->data[pos_i][pos_j] = *j;
			++pos_j;
		}
		++pos_i;
		pos_j = 0;
	}
}

template<typename T, std::size_t m, std::size_t n>
constexpr std::size_t Matrix<T, m, n>::get_m() const noexcept
{
	return m;
}

template<typename T, std::size_t m, std::size_t n>
constexpr std::size_t Matrix<T, m, n>::get_n() const noexcept
{
	return n;
}

template<typename T, std::size_t m, std::size_t n>
constexpr std::array<T, n> &Matrix<T, m, n>::operator[](
	const std::size_t pos) noexcept
{
	return this->data[pos];
}

template<typename T, std::size_t m, std::size_t n>
constexpr const std::array<T, n> &Matrix<T, m, n>::operator[](
	const std::size_t pos) const noexcept
{
	return this->data[pos];
}

template<typename T, std::size_t m, std::size_t n>
constexpr std::array<T, n> &Matrix<T, m, n>::at(const std::size_t pos)
{
	if (pos >= m) {
		throw std::out_of_range("Index out of range.");
	}

	return this->data.at(pos);
}

template<typename T, std::size_t m, std::size_t n>
constexpr const std::array<T, n> &Matrix<T, m, n>::at(
	const std::size_t pos) const
{
	if (pos >= m) {
		throw std::out_of_range("Index out of range.");
	}

	return this->data.at(pos);
}

template<typename T, std::size_t m, std::size_t n>
constexpr bool Matrix<T, m, n>::fuzzy_equal(
	const Matrix<T, m, n> &mat, T fuzz) const noexcept
{
	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			if (std::abs((*this)[i][j] - mat[i][j]) > fuzz) {
				return false;
			}
		}
	}

	return true;
}

template<typename T, std::size_t m, std::size_t n>
constexpr bool Matrix<T, m, n>::operator==(const Matrix<T, m, n> &mat) const
	noexcept
{
	return this->fuzzy_equal(mat, 0);
}

template<typename T, std::size_t m, std::size_t n>
constexpr bool Matrix<T, m, n>::operator!=(const Matrix<T, m, n> &mat) const
	noexcept
{
	return !(*this == mat);
}

template<typename T, std::size_t m, std::size_t n>
constexpr void Matrix<T, m, n>::operator+=(T scalar) noexcept
{
	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			(*this)[i][j] += scalar;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
constexpr void Matrix<T, m, n>::operator-=(T scalar) noexcept
{
	(*this) += -1 * scalar;
}

template<typename T, std::size_t m, std::size_t n>
constexpr void Matrix<T, m, n>::operator*=(T scalar) noexcept
{
	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			(*this)[i][j] *= scalar;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
constexpr void Matrix<T, m, n>::operator/=(T scalar) noexcept
{
	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			(*this)[i][j] /= scalar;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
constexpr void Matrix<T, m, n>::operator+=(const Matrix<T, m, n> &mat) noexcept
{
	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			(*this)[i][j] += mat[i][j];
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
constexpr void Matrix<T, m, n>::operator-=(const Matrix<T, m, n> &mat) noexcept
{
	(*this) += mat * static_cast<T>(-1);
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> Matrix<T, m, n>::operator+(
	const Matrix<T, m, n> &mat) const noexcept
{
	Matrix<T, m, n> copy(*this);
	copy += mat;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> Matrix<T, m, n>::operator-(
	const Matrix<T, m, n> &mat) const noexcept
{
	Matrix<T, m, n> copy(*this);
	copy -= mat;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
template<std::size_t p>
constexpr Matrix<T, m, p> Matrix<T, m, n>::operator*(
	const Matrix<T, n, p> &mat) const noexcept
{
	Matrix<T, m, p> product;

	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < p; ++j) {
			for (std::size_t k = 0; k < n; ++k) {
				product[i][j] += (*this)[i][k] * mat[k][j];
			}
		}
	}

	return product;
}

template<typename T, std::size_t m, std::size_t n>
template<std::size_t p>
constexpr Matrix<T, m, n + p> Matrix<T, m, n>::horizontal_augment(
	const Matrix<T, m, p> &mat) const noexcept
{
	Matrix<T, m, n + p> augmented;

	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			augmented[i][j] = (*this)[i][j];
		}

		for (std::size_t j = n; j < n + p; ++j) {
			augmented[i][j] = mat[i][j - n];
		}
	}

	return augmented;
}

template<typename T, std::size_t m, std::size_t n>
template<std::size_t p>
constexpr Matrix<T, m + p, n> Matrix<T, m, n>::vertical_augment(
	const Matrix<T, p, n> &mat) const noexcept
{
	Matrix<T, m + p, n> augmented;

	for (std::size_t i = 0; i < n; ++i) {
		for (std::size_t j = 0; j < m; ++j) {
			augmented[j][i] = (*this)[j][i];
		}

		for (std::size_t j = m; j < m + p; ++j) {
			augmented[j][i] = mat[j - m][i];
		}
	}

	return augmented;
}

template<typename T, std::size_t m, std::size_t n>
template<std::size_t y, std::size_t x, std::size_t p, std::size_t q>
constexpr Matrix<T, p, q> Matrix<T, m, n>::slice() const noexcept
{
	Matrix<T, p, q> mat;

	for (std::size_t i = y; i < m && i - y < p; ++i) {
		for (std::size_t j = x; j < n && j - x < q; ++j) {
			mat[i - y][j - x] = (*this)[i][j];
		}
	}

	return mat;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, n, m> Matrix<T, m, n>::transpose() const noexcept
{
	Matrix<T, n, m> transposed;

	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			transposed[j][i] = (*this)[i][j];
		}
	}

	return transposed;
}

template<typename T, std::size_t m, std::size_t n>
constexpr std::optional<Matrix<T, m, n>> Matrix<T, m, n>::inverse() const
	noexcept
{
	static_assert(
		m == n, "Inverse matrix is only defined for square matrices.");

	const auto identity = Matrix<T, m, n>::get_identity();
	auto augmented = this->horizontal_augment(identity);

	for (std::size_t i = 0; i < m; ++i) {
		/**
		 *  Find the diagonal element closest to -1 or 1.
		 * 	And if it isn't in the current row swap it.
		 *  This is done to make the inverse mroe numerically stable.
		 *  Also check if that the row may not be zero since that would
		 *  result in a divide by zero error.
		 */
		std::size_t min_row_index = i;
		std::optional<T> closest_to_one;

		for (std::size_t j = min_row_index; j < m; ++j) {
			if (std::abs(augmented[j][i])
				<= std::numeric_limits<T>::min()) {
				continue;
			}

			const T abs_value =
				std::abs(1 - std::abs(augmented[j][i]));

			if (!closest_to_one.has_value()
				|| abs_value < closest_to_one.value()) {
				closest_to_one = abs_value;
				min_row_index = j;
			}
		}

		if (!closest_to_one.has_value()) {
			return std::nullopt;
		}

		if (min_row_index != i) {
			for (std::size_t j = 0; j < 2 * m; ++j) {
				T tmp = augmented[i][j];
				augmented[i][j] = augmented[min_row_index][j];
				augmented[min_row_index][j] = tmp;
			}
		}

		// Divide the current row by a scalar so that the current
		// diagonal element will be 1.
		const auto divisor = augmented[i][i];

		for (std::size_t j = 0; j < 2 * m; ++j) {
			augmented[i][j] /= divisor;
		}

		// Set all other rows to 0 by using the row that is 1.
		for (std::size_t j = 0; j < m; ++j) {
			if (i == j) {
				continue;
			}

			const auto multiplier = augmented[j][i];

			for (std::size_t k = 0; k < 2 * m; ++k) {
				augmented[j][k] -= multiplier * augmented[i][k];
			}
		}
	}

	/**
	 * Check if the inversed matrix times the original matrix is equal
	 * to the identity matrix. The equal precision is the precision
	 * when the identity matrix is considered equal.
	 */
	const auto inversed = augmented.template slice<0, m, m, m>();
	const auto identity_check = inversed * (*this);
	const auto equal_precision = 0.0000000001;

	if (!identity_check.fuzzy_equal(identity, equal_precision)) {
		return std::nullopt;
	}

	return inversed;
}

template<typename T, std::size_t m, std::size_t n>
constexpr T Matrix<T, m, n>::get_magnitude() const noexcept
{
	static_assert(m == 1 || n == 1,
		"Magnitude of a matrix is only defined for column and row "
		"matrices.");

	T magnitude = 0;

	if constexpr (m == 1) // NOLINT
	{
		magnitude = static_cast<T>(
			std::sqrt(((*this) * this->transpose())[0][0]));
	} else {
		magnitude = static_cast<T>(
			std::sqrt((this->transpose() * (*this))[0][0]));
	}

	return magnitude;
}

template<typename T, std::size_t m, std::size_t n>
std::string Matrix<T, m, n>::str() const
{
	std::stringstream ss;
	ss << std::setprecision(9) << std::setw(6) << std::fixed;

	for (std::size_t i = 0; i < m; ++i) {
		for (std::size_t j = 0; j < n; ++j) {
			ss << (*this)[i][j] << "\t";
		}
		ss << std::endl;
	}

	return ss.str();
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator+(const Matrix<T, m, n> &lhs, T rhs) noexcept
{
	Matrix<T, m, n> copy(lhs);
	copy += rhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator+(T lhs, const Matrix<T, m, n> &rhs) noexcept
{
	Matrix<T, m, n> copy(rhs);
	copy += lhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator-(const Matrix<T, m, n> &lhs, T rhs) noexcept
{
	Matrix<T, m, n> copy(lhs);
	copy -= rhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator-(T lhs, const Matrix<T, m, n> &rhs) noexcept
{
	Matrix<T, m, n> copy(rhs);
	copy -= lhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator*(const Matrix<T, m, n> &lhs, T rhs) noexcept
{
	Matrix<T, m, n> copy(lhs);
	copy *= rhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator*(T lhs, const Matrix<T, m, n> &rhs) noexcept
{
	Matrix<T, m, n> copy(rhs);
	copy *= lhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator/(const Matrix<T, m, n> &lhs, T rhs) noexcept
{
	Matrix<T, m, n> copy(lhs);
	copy /= rhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
constexpr Matrix<T, m, n> operator/(T lhs, const Matrix<T, m, n> &rhs) noexcept
{
	Matrix<T, m, n> copy(rhs);
	copy /= lhs;
	return copy;
}

template<typename T, std::size_t m, std::size_t n>
std::ostream &operator<<(std::ostream &os, const Matrix<T, m, n> &mat)
{
	return os << mat.str();
}
