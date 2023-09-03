/*
 *  Profiling infrastructure declarations.
 *
 *  This file is based on gcc-internal definitions. Data structures are
 *  defined to be compatible with gcc counterparts. For a better
 *  understanding, refer to gcc source: gcc/gcov-io.h.
 *
 *    Copyright IBM Corp. 2009
 *    Author(s): Peter Oberparleiter <oberpar@linux.vnet.ibm.com>
 *
 *    Uses gcc-internal data definitions.
 */

#ifndef GCOV_H
#define GCOV_H GCOV_H

#include <linux/types.h>

/*
 * GCC 4.6 drops the 'name' field from 'struct gcov_fn_info'.
 */
#if __GNUC__ < 4 || (__GNUC__ == 4 && __GNUC_MINOR__ < 6)
#define GCOV_FN_INFO_HAS_NAME_FIELD
#endif

/*
 * Profiling data types used for at least gcc 4.4 and 4.6 - these are defined by
 * gcc and need to be kept as close to the original definition as possible to
 * remain compatible.
 */
#define GCOV_COUNTERS		10
#define GCOV_DATA_MAGIC		((unsigned int) 0x67636461)
#define GCOV_TAG_FUNCTION	((unsigned int) 0x01000000)
#define GCOV_TAG_FUNCTION_LENGTH 3
#define GCOV_TAG_COUNTER_BASE	((unsigned int) 0x01a10000)
#define GCOV_TAG_FOR_COUNTER(count)					\
	(GCOV_TAG_COUNTER_BASE + ((unsigned int) (count) << 17))

#if BITS_PER_LONG >= 64
typedef long gcov_type;
#else
typedef long long gcov_type;
#endif

/*
 * Source module info. The data structure is used in both runtime and
 * profile-use phase.
 */
struct gcov_module_info {
	unsigned int ident;
/*
 * This is overloaded to mean two things:
 * (1) means FDO/LIPO in instrumented binary.
 * (2) means IS_PRIMARY in persistent file or memory copy used in profile-use.
 */
	unsigned int is_primary;
	unsigned int is_exported;
	unsigned int lang;
	char *da_filename;
	char *source_filename;
	unsigned int num_quote_paths;
	unsigned int num_bracket_paths;
	unsigned int num_cpp_defines;
	unsigned int num_cpp_includes;
	unsigned int num_cl_args;
	char *string_array[1];
};


/**
 * struct gcov_fn_info - profiling meta data per function
 * @ident: object file-unique function identifier
 * @lineno_checksum: function lineno checksum
 * @cfg_checksum: function cfg checksum
 * @dc_offset: direct call offset
 * @name: function name
 * @n_ctrs: number of values per counter type belonging to this function
 *
 * This data is generated by gcc during compilation and doesn't change
 * at run-time.
 */
struct gcov_fn_info {
	unsigned int ident;
	unsigned int lineno_checksum;
	unsigned int cfg_checksum;
	unsigned int dc_offset;
#ifdef GCOV_FN_INFO_HAS_NAME_FIELD
	const char   *name;
#endif
	unsigned int n_ctrs[0];
};

/**
 * struct gcov_ctr_info - profiling data per counter type
 * @num: number of counter values for this type
 * @values: array of counter values for this type
 * @merge: merge function for counter values of this type (unused)
 *
 * This data is generated by gcc during compilation and doesn't change
 * at run-time with the exception of the values array.
 */
struct gcov_ctr_info {
	unsigned int	num;
	gcov_type	*values;
	void		(*merge)(gcov_type *, unsigned int);
};

/**
 * struct gcov_info - profiling data per object file
 * @version: gcov version magic indicating the gcc version used for compilation
 * @modinfo: additional module information
 * @next: list head for a singly-linked list
 * @stamp: time stamp
 * @filename: name of the associated gcov data file
 * @eof_pos: end position of profile data
 * @n_functions: number of instrumented functions
 * @functions: function data
 * @ctr_mask: mask specifying which counter types are active
 * @counts: counter data per counter type
 *
 * This data is generated by gcc during compilation and doesn't change
 * at run-time with the exception of the next pointer.
 */
struct gcov_info {
	unsigned int			version;
	struct gcov_module_info		*mod_info;
	struct gcov_info		*next;
	unsigned int			stamp;
	const char			*filename;
	unsigned int			eof_pos;
	unsigned int			n_functions;
	const struct gcov_fn_info	*functions;
	unsigned int			ctr_mask;
	struct gcov_ctr_info		counts[0];
};

/* Base interface. */
enum gcov_action {
	GCOV_ADD,
	GCOV_REMOVE,
};

void gcov_event(enum gcov_action action, struct gcov_info *info);
void gcov_enable_events(void);

/* Iterator control. */
struct seq_file;
struct gcov_iterator;

struct gcov_iterator *gcov_iter_new(struct gcov_info *info);
void gcov_iter_free(struct gcov_iterator *iter);
void gcov_iter_start(struct gcov_iterator *iter);
int gcov_iter_next(struct gcov_iterator *iter);
int gcov_iter_write(struct gcov_iterator *iter, struct seq_file *seq);
struct gcov_info *gcov_iter_get_info(struct gcov_iterator *iter);

/* gcov_info control. */
void gcov_info_reset(struct gcov_info *info);
int gcov_info_is_compatible(struct gcov_info *info1, struct gcov_info *info2);
void gcov_info_add(struct gcov_info *dest, struct gcov_info *source);
struct gcov_info *gcov_info_dup(struct gcov_info *info);
void gcov_info_free(struct gcov_info *info);

struct gcov_link {
	enum {
		OBJ_TREE,
		SRC_TREE,
	} dir;
	const char *ext;
};
extern const struct gcov_link gcov_link[];

#endif /* GCOV_H */
