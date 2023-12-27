#ifndef MY_STRING_FUNCTIONS_H
#define MY_STRING_FUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations for assembly string functions

/**
 * Calculates the length of a string.
 *
 * @param str Pointer to the string.
 * @return Length of the string.
 */
int my_strlen(const char *str);

/**
 * Copies a string into another string.
 *
 * @param dest Destination string.
 * @param src Source string.
 * @return Pointer to the destination string.
 */
char *my_strcpy(char *dest, const char *src);

/**
 * Compares two strings.
 *
 * @param str1 First string.
 * @param str2 Second string.
 * @return An integer less than, equal to, or greater than zero if str1 is found,
 * respectively, to be less than, to match, or be greater than str2.
 */
int my_strcmp(const char *str1, const char *str2);

/**
 * Compares two strings with a length limit.
 *
 * @param str1 First string.
 * @param str2 Second string.
 * @param n Number of characters to compare.
 * @return An integer less than, equal to, or greater than zero if the first n characters
 * of str1 are found, respectively, to be less than, to match, or be greater than the first n
 * characters of str2.
 */
int my_strncmp(const char *str1, const char *str2, size_t n);

/**
 * Concatenates two strings.
 *
 * @param dest Destination string.
 * @param src Source string.
 * @return Pointer to the destination string.
 */
char *my_strcat(char *dest, const char *src);

/**
 * Searches for the first occurrence of a character in a string.
 *
 * @param str String to be searched.
 * @param ch Character to search for.
 * @return Pointer to the first occurrence of the character in the string, or NULL if the character is not found.
 */
char *my_strchr(const char *str, char ch);

/**
 * Searches for the last occurrence of a character in a string.
 *
 * @param str String to be searched.
 * @param ch Character to search for.
 * @return Pointer to the last occurrence of the character in the string, or NULL if the character is not found.
 */
char *my_strrchr(const char *str, char ch);

/**
 * Finds the index of the first occurrence of a character in a string.
 *
 * @param str String to be searched.
 * @param ch Character to search for.
 * @return Index of the first occurrence of the character in the string, or -1 if the character is not found.
 */
int my_findchr(const char *str, char ch);

/**
 * Searches for a substring within a string.
 *
 * @param str The string to be searched.
 * @param substr The substring to find.
 * @return Pointer to the beginning of the located substring, or NULL if the substring is not found.
 */
char *my_strstr(const char *str, const char *substr);

#ifdef __cplusplus
}
#endif

#endif // MY_STRING_FUNCTIONS_H
