/** Moved implementation from cpptoml.h
 *
 * Done by Michael Bentley to hopefully improve compile time.
 * Just the big stuff that's not templatized.
 *
 * The order of functions here are the order they appear in cpptoml.h
 */

#include "cpptoml/cpptoml.h"

namespace cpptoml
{

std::ostream& operator<<(std::ostream& os, const local_time& ltime)
{
    fill_guard g{os};
    os.fill('0');

    using std::setw;
    os << setw(2) << ltime.hour << ":" << setw(2) << ltime.minute << ":"
       << setw(2) << ltime.second;

    if (ltime.microsecond > 0)
    {
        os << ".";
        int power = 100000;
        for (int curr_us = ltime.microsecond; curr_us; power /= 10)
        {
            auto num = curr_us / power;
            os << num;
            curr_us -= num * power;
        }
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const zone_offset& zo)
{
    fill_guard g{os};
    os.fill('0');

    using std::setw;

    if (zo.hour_offset != 0 || zo.minute_offset != 0)
    {
        if (zo.hour_offset > 0)
        {
            os << "+";
        }
        else
        {
            os << "-";
        }
        os << setw(2) << std::abs(zo.hour_offset) << ":" << setw(2)
           << std::abs(zo.minute_offset);
    }
    else
    {
        os << "Z";
    }

    return os;
}

template <>
std::shared_ptr<value<double>> base::as()
{
#if defined(CPPTOML_NO_RTTI)
    if (type() == base_type::FLOAT)
        return std::static_pointer_cast<value<double>>(shared_from_this());

    if (type() == base_type::INT)
    {
        auto v = std::static_pointer_cast<value<int64_t>>(shared_from_this());
        return make_value<double>(static_cast<double>(v->get()));
    }
#else
    if (auto v = std::dynamic_pointer_cast<value<double>>(shared_from_this()))
        return v;

    if (auto v = std::dynamic_pointer_cast<value<int64_t>>(shared_from_this()))
        return make_value<double>(static_cast<double>(v->get()));
#endif

    return nullptr;
}

template <>
std::shared_ptr<const value<double>> base::as() const
{
#if defined(CPPTOML_NO_RTTI)
    if (type() == base_type::FLOAT)
        return std::static_pointer_cast<const value<double>>(
            shared_from_this());

    if (type() == base_type::INT)
    {
        auto v = as<int64_t>();
        // the below has to be a non-const value<double> due to a bug in
        // libc++: https://llvm.org/bugs/show_bug.cgi?id=18843
        return make_value<double>(static_cast<double>(v->get()));
    }
#else
    if (auto v
        = std::dynamic_pointer_cast<const value<double>>(shared_from_this()))
        return v;

    if (auto v = as<int64_t>())
    {
        // the below has to be a non-const value<double> due to a bug in
        // libc++: https://llvm.org/bugs/show_bug.cgi?id=18843
        return make_value<double>(static_cast<double>(v->get()));
    }
#endif

    return nullptr;
}

template <>
typename array_of_trait<array>::return_type
array::get_array_of<array>() const
{
    std::vector<std::shared_ptr<array>> result;
    result.reserve(values_.size());

    for (const auto& val : values_)
    {
        if (auto v = val->as_array())
            result.push_back(v);
        else
            return {};
    }

    return {std::move(result)};
}

std::vector<std::string> table::split(const std::string &value,
                                      char separator) const
{
    std::vector<std::string> result;
    std::string::size_type p = 0;
    std::string::size_type q;
    while ((q = value.find(separator, p)) != std::string::npos)
    {
        result.emplace_back(value, p, q - p);
        p = q + 1;
    }
    result.emplace_back(value, p);
    return result;
}

bool table::resolve_qualified(const std::string& key,
                              std::shared_ptr<base>* p) const
{
    auto parts = split(key, '.');
    auto last_key = parts.back();
    parts.pop_back();

    auto cur_table = this;
    for (const auto& part : parts)
    {
        cur_table = cur_table->get_table(part).get();
        if (!cur_table)
        {
            if (!p)
                return false;

            throw std::out_of_range{key + " is not a valid key"};
        }
    }

    if (!p)
        return cur_table->map_.count(last_key) != 0;

    *p = cur_table->map_.at(last_key);
    return true;
}

template <>
typename array_of_trait<array>::return_type
table::get_array_of<array>(const std::string& key) const
{
    if (auto v = get_array(key))
    {
        std::vector<std::shared_ptr<array>> result;
        result.reserve(v->get().size());

        for (const auto& b : v->get())
        {
            if (auto val = b->as_array())
                result.push_back(val);
            else
                return {};
        }

        return {std::move(result)};
    }

    return {};
}

template <>
typename array_of_trait<array>::return_type
table::get_qualified_array_of<array>(const std::string& key) const
{
    if (auto v = get_array_qualified(key))
    {
        std::vector<std::shared_ptr<array>> result;
        result.reserve(v->get().size());

        for (const auto& b : v->get())
        {
            if (auto val = b->as_array())
                result.push_back(val);
            else
                return {};
        }

        return {std::move(result)};
    }

    return {};
}

namespace detail
{
inline std::istream& getline(std::istream& input, std::string& line)
{
    line.clear();

    std::istream::sentry sentry{input, true};
    auto sb = input.rdbuf();

    while (true)
    {
        auto c = sb->sbumpc();
        if (c == '\r')
        {
            if (sb->sgetc() == '\n')
                c = sb->sbumpc();
        }

        if (c == '\n')
            return input;

        if (c == std::istream::traits_type::eof())
        {
            if (line.empty())
                input.setstate(std::ios::eofbit);
            return input;
        }

        line.push_back(static_cast<char>(c));
    }
}
} // namespace detail

std::shared_ptr<table> parser::parse()
{
    std::shared_ptr<table> root = make_table();

    table* curr_table = root.get();

    while (detail::getline(input_, line_))
    {
        line_number_++;
        auto it = line_.begin();
        auto end = line_.end();
        consume_whitespace(it, end);
        if (it == end || *it == '#')
            continue;
        if (*it == '[')
        {
            curr_table = root.get();
            parse_table(it, end, curr_table);
        }
        else
        {
            parse_key_value(it, end, curr_table);
            consume_whitespace(it, end);
            eol_or_comment(it, end);
        }
    }
    return root;
}

void parser::parse_table(std::string::iterator& it,
                         const std::string::iterator& end, table*& curr_table)
{
    // remove the beginning keytable marker
    ++it;
    if (it == end)
        throw_parse_exception("Unexpected end of table");
    if (*it == '[')
        parse_table_array(it, end, curr_table);
    else
        parse_single_table(it, end, curr_table);
}

void parser::parse_single_table(std::string::iterator& it,
                                const std::string::iterator& end,
                                table*& curr_table)
{
    if (it == end || *it == ']')
        throw_parse_exception("Table name cannot be empty");

    std::string full_table_name;
    bool inserted = false;

    auto key_end = [](char c) { return c == ']'; };

    auto key_part_handler = [&](const std::string& part) {
        if (part.empty())
            throw_parse_exception("Empty component of table name");

        if (!full_table_name.empty())
            full_table_name += '.';
        full_table_name += part;

        if (curr_table->contains(part))
        {
#if !defined(__PGI)
            auto b = curr_table->get(part);
#else
            // Workaround for PGI compiler
            std::shared_ptr<base> b = curr_table->get(part);
#endif
            if (b->is_table())
                curr_table = static_cast<table*>(b.get());
            else if (b->is_table_array())
                curr_table = std::static_pointer_cast<table_array>(b)
                                 ->get()
                                 .back()
                                 .get();
            else
                throw_parse_exception("Key " + full_table_name
                                      + "already exists as a value");
        }
        else
        {
            inserted = true;
            curr_table->insert(part, make_table());
            curr_table = static_cast<table*>(curr_table->get(part).get());
        }
    };

    key_part_handler(parse_key(it, end, key_end, key_part_handler));

    if (it == end)
        throw_parse_exception(
            "Unterminated table declaration; did you forget a ']'?");

    if (*it != ']')
    {
        std::string errmsg{"Unexpected character in table definition: "};
        errmsg += '"';
        errmsg += *it;
        errmsg += '"';
        throw_parse_exception(errmsg);
    }

    // table already existed
    if (!inserted)
    {
        auto is_value
            = [](const std::pair<const std::string&,
                                 const std::shared_ptr<base>&>& p) {
                  return p.second->is_value();
              };

        // if there are any values, we can't add values to this table
        // since it has already been defined. If there aren't any
        // values, then it was implicitly created by something like
        // [a.b]
        if (curr_table->empty()
            || std::any_of(curr_table->begin(), curr_table->end(),
                           is_value))
        {
            throw_parse_exception("Redefinition of table "
                                  + full_table_name);
        }
    }

    ++it;
    consume_whitespace(it, end);
    eol_or_comment(it, end);
}

void parser::parse_table_array(std::string::iterator& it,
                               const std::string::iterator& end, table*& curr_table)
{
    ++it;
    if (it == end || *it == ']')
        throw_parse_exception("Table array name cannot be empty");

    auto key_end = [](char c) { return c == ']'; };

    std::string full_ta_name;
    auto key_part_handler = [&](const std::string& part) {
        if (part.empty())
            throw_parse_exception("Empty component of table array name");

        if (!full_ta_name.empty())
            full_ta_name += '.';
        full_ta_name += part;

        if (curr_table->contains(part))
        {
#if !defined(__PGI)
            auto b = curr_table->get(part);
#else
            // Workaround for PGI compiler
            std::shared_ptr<base> b = curr_table->get(part);
#endif

            // if this is the end of the table array name, add an
            // element to the table array that we just looked up,
            // provided it was not declared inline
            if (it != end && *it == ']')
            {
                if (!b->is_table_array())
                {
                    throw_parse_exception("Key " + full_ta_name
                                          + " is not a table array");
                }

                auto v = b->as_table_array();

                if (v->is_inline())
                {
                    throw_parse_exception("Static array " + full_ta_name
                                          + " cannot be appended to");
                }

                v->get().push_back(make_table());
                curr_table = v->get().back().get();
            }
            // otherwise, just keep traversing down the key name
            else
            {
                if (b->is_table())
                    curr_table = static_cast<table*>(b.get());
                else if (b->is_table_array())
                    curr_table = std::static_pointer_cast<table_array>(b)
                                     ->get()
                                     .back()
                                     .get();
                else
                    throw_parse_exception("Key " + full_ta_name
                                          + " already exists as a value");
            }
        }
        else
        {
            // if this is the end of the table array name, add a new
            // table array and a new table inside that array for us to
            // add keys to next
            if (it != end && *it == ']')
            {
                curr_table->insert(part, make_table_array());
                auto arr = std::static_pointer_cast<table_array>(
                    curr_table->get(part));
                arr->get().push_back(make_table());
                curr_table = arr->get().back().get();
            }
            // otherwise, create the implicitly defined table and move
            // down to it
            else
            {
                curr_table->insert(part, make_table());
                curr_table
                    = static_cast<table*>(curr_table->get(part).get());
            }
        }
    };

    key_part_handler(parse_key(it, end, key_end, key_part_handler));

    // consume the last "]]"
    auto eat = make_consumer(it, end, [this]() {
        throw_parse_exception("Unterminated table array name");
    });
    eat(']');
    eat(']');

    consume_whitespace(it, end);
    eol_or_comment(it, end);
}

void parser::parse_key_value(std::string::iterator& it, std::string::iterator& end,
                             table* curr_table)
{
    auto key_end = [](char c) { return c == '='; };

    auto key_part_handler = [&](const std::string& part) {
        // two cases: this key part exists already, in which case it must
        // be a table, or it doesn't exist in which case we must create
        // an implicitly defined table
        if (curr_table->contains(part))
        {
            auto val = curr_table->get(part);
            if (val->is_table())
            {
                curr_table = static_cast<table*>(val.get());
            }
            else
            {
                throw_parse_exception("Key " + part
                                      + " already exists as a value");
            }
        }
        else
        {
            auto newtable = make_table();
            curr_table->insert(part, newtable);
            curr_table = newtable.get();
        }
    };

    auto key = parse_key(it, end, key_end, key_part_handler);

    if (curr_table->contains(key))
        throw_parse_exception("Key " + key + " already present");
    if (it == end || *it != '=')
        throw_parse_exception("Value must follow after a '='");
    ++it;
    consume_whitespace(it, end);
    curr_table->insert(key, parse_value(it, end));
    consume_whitespace(it, end);
}

std::string parser::parse_simple_key(std::string::iterator& it,
                                     const std::string::iterator& end)
{
    consume_whitespace(it, end);

    if (it == end)
        throw_parse_exception("Unexpected end of key (blank key?)");

    if (*it == '"' || *it == '\'')
    {
        return string_literal(it, end, *it);
    }
    else
    {
        auto bke = std::find_if(it, end, [](char c) {
            return c == '.' || c == '=' || c == ']';
        });
        return parse_bare_key(it, bke);
    }
}

std::string parser::parse_bare_key(std::string::iterator& it,
                                   const std::string::iterator& end)
{
    if (it == end)
    {
        throw_parse_exception("Bare key missing name");
    }

    auto key_end = end;
    --key_end;
    consume_backwards_whitespace(key_end, it);
    ++key_end;
    std::string key{it, key_end};

    if (std::find(it, key_end, '#') != key_end)
    {
        throw_parse_exception("Bare key " + key + " cannot contain #");
    }

    if (std::find_if(it, key_end,
                     [](char c) { return c == ' ' || c == '\t'; })
        != key_end)
    {
        throw_parse_exception("Bare key " + key
                              + " cannot contain whitespace");
    }

    if (std::find_if(it, key_end,
                     [](char c) { return c == '[' || c == ']'; })
        != key_end)
    {
        throw_parse_exception("Bare key " + key
                              + " cannot contain '[' or ']'");
    }

    it = end;
    return key;
}

std::shared_ptr<base> parser::parse_value(std::string::iterator& it,
                                          std::string::iterator& end)
{
    parse_type type = determine_value_type(it, end);
    switch (type)
    {
        case parse_type::STRING:
            return parse_string(it, end);
        case parse_type::LOCAL_TIME:
            return parse_time(it, end);
        case parse_type::LOCAL_DATE:
        case parse_type::LOCAL_DATETIME:
        case parse_type::OFFSET_DATETIME:
            return parse_date(it, end);
        case parse_type::INT:
        case parse_type::FLOAT:
            return parse_number(it, end);
        case parse_type::BOOL:
            return parse_bool(it, end);
        case parse_type::ARRAY:
            return parse_array(it, end);
        case parse_type::INLINE_TABLE:
            return parse_inline_table(it, end);
        default:
            throw_parse_exception("Failed to parse value");
    }
}

parser::parse_type parser::determine_value_type(const std::string::iterator& it,
                                                const std::string::iterator& end)
{
    if (it == end)
    {
        throw_parse_exception("Failed to parse value type");
    }
    if (*it == '"' || *it == '\'')
    {
        return parse_type::STRING;
    }
    else if (is_time(it, end))
    {
        return parse_type::LOCAL_TIME;
    }
    else if (auto dtype = date_type(it, end))
    {
        return *dtype;
    }
    else if (is_number(*it) || *it == '-' || *it == '+'
             || (*it == 'i' && it + 1 != end && it[1] == 'n'
                 && it + 2 != end && it[2] == 'f')
             || (*it == 'n' && it + 1 != end && it[1] == 'a'
                 && it + 2 != end && it[2] == 'n'))
    {
        return determine_number_type(it, end);
    }
    else if (*it == 't' || *it == 'f')
    {
        return parse_type::BOOL;
    }
    else if (*it == '[')
    {
        return parse_type::ARRAY;
    }
    else if (*it == '{')
    {
        return parse_type::INLINE_TABLE;
    }
    throw_parse_exception("Failed to parse value type");
}

parser::parse_type parser::determine_number_type(const std::string::iterator& it,
                                                 const std::string::iterator& end)
{
    // determine if we are an integer or a float
    auto check_it = it;
    if (*check_it == '-' || *check_it == '+')
        ++check_it;

    if (check_it == end)
        throw_parse_exception("Malformed number");

    if (*check_it == 'i' || *check_it == 'n')
        return parse_type::FLOAT;

    while (check_it != end && is_number(*check_it))
        ++check_it;
    if (check_it != end && *check_it == '.')
    {
        ++check_it;
        while (check_it != end && is_number(*check_it))
            ++check_it;
        return parse_type::FLOAT;
    }
    else
    {
        return parse_type::INT;
    }
}

std::shared_ptr<value<std::string>> parser::parse_string(std::string::iterator& it,
                                                         std::string::iterator& end)
{
    auto delim = *it;
    assert(delim == '"' || delim == '\'');

    // end is non-const here because we have to be able to potentially
    // parse multiple lines in a string, not just one
    auto check_it = it;
    ++check_it;
    if (check_it != end && *check_it == delim)
    {
        ++check_it;
        if (check_it != end && *check_it == delim)
        {
            it = ++check_it;
            return parse_multiline_string(it, end, delim);
        }
    }
    return make_value<std::string>(string_literal(it, end, delim));
}

std::shared_ptr<value<std::string>>
parser::parse_multiline_string(std::string::iterator& it,
                               std::string::iterator& end, char delim)
{
    std::stringstream ss;

    auto is_ws = [](char c) { return c == ' ' || c == '\t'; };

    bool consuming = false;
    std::shared_ptr<value<std::string>> ret;

    auto handle_line = [&](std::string::iterator& local_it,
                           std::string::iterator& local_end) {
        if (consuming)
        {
            local_it = std::find_if_not(local_it, local_end, is_ws);

            // whole line is whitespace
            if (local_it == local_end)
                return;
        }

        consuming = false;

        while (local_it != local_end)
        {
            // handle escaped characters
            if (delim == '"' && *local_it == '\\')
            {
                auto check = local_it;
                // check if this is an actual escape sequence or a
                // whitespace escaping backslash
                ++check;
                consume_whitespace(check, local_end);
                if (check == local_end)
                {
                    consuming = true;
                    break;
                }

                ss << parse_escape_code(local_it, local_end);
                continue;
            }

            // if we can end the string
            if (std::distance(local_it, local_end) >= 3)
            {
                auto check = local_it;
                // check for """
                if (*check++ == delim && *check++ == delim
                    && *check++ == delim)
                {
                    local_it = check;
                    ret = make_value<std::string>(ss.str());
                    break;
                }
            }

            ss << *local_it++;
        }
    };

    // handle the remainder of the current line
    handle_line(it, end);
    if (ret)
        return ret;

    // start eating lines
    while (detail::getline(input_, line_))
    {
        ++line_number_;

        it = line_.begin();
        end = line_.end();

        handle_line(it, end);

        if (ret)
            return ret;

        if (!consuming)
            ss << std::endl;
    }

    throw_parse_exception("Unterminated multi-line basic string");
}

std::string parser::string_literal(std::string::iterator& it,
                                   const std::string::iterator& end, char delim)
{
    ++it;
    std::string val;
    while (it != end)
    {
        // handle escaped characters
        if (delim == '"' && *it == '\\')
        {
            val += parse_escape_code(it, end);
        }
        else if (*it == delim)
        {
            ++it;
            consume_whitespace(it, end);
            return val;
        }
        else
        {
            val += *it++;
        }
    }
    throw_parse_exception("Unterminated string literal");
}

std::string parser::parse_escape_code(std::string::iterator& it,
                                      const std::string::iterator& end)
{
    ++it;
    if (it == end)
        throw_parse_exception("Invalid escape sequence");
    char value;
    if (*it == 'b')
    {
        value = '\b';
    }
    else if (*it == 't')
    {
        value = '\t';
    }
    else if (*it == 'n')
    {
        value = '\n';
    }
    else if (*it == 'f')
    {
        value = '\f';
    }
    else if (*it == 'r')
    {
        value = '\r';
    }
    else if (*it == '"')
    {
        value = '"';
    }
    else if (*it == '\\')
    {
        value = '\\';
    }
    else if (*it == 'u' || *it == 'U')
    {
        return parse_unicode(it, end);
    }
    else
    {
        throw_parse_exception("Invalid escape sequence");
    }
    ++it;
    return std::string(1, value);
}

std::string parser::parse_unicode(std::string::iterator& it,
                                  const std::string::iterator& end)
{
    bool large = *it++ == 'U';
    auto codepoint = parse_hex(it, end, large ? 0x10000000 : 0x1000);

    if ((codepoint > 0xd7ff && codepoint < 0xe000) || codepoint > 0x10ffff)
    {
        throw_parse_exception(
            "Unicode escape sequence is not a Unicode scalar value");
    }

    std::string result;
    // See Table 3-6 of the Unicode standard
    if (codepoint <= 0x7f)
    {
        // 1-byte codepoints: 00000000 0xxxxxxx
        // repr: 0xxxxxxx
        result += static_cast<char>(codepoint & 0x7f);
    }
    else if (codepoint <= 0x7ff)
    {
        // 2-byte codepoints: 00000yyy yyxxxxxx
        // repr: 110yyyyy 10xxxxxx
        //
        // 0x1f = 00011111
        // 0xc0 = 11000000
        //
        result += static_cast<char>(0xc0 | ((codepoint >> 6) & 0x1f));
        //
        // 0x80 = 10000000
        // 0x3f = 00111111
        //
        result += static_cast<char>(0x80 | (codepoint & 0x3f));
    }
    else if (codepoint <= 0xffff)
    {
        // 3-byte codepoints: zzzzyyyy yyxxxxxx
        // repr: 1110zzzz 10yyyyyy 10xxxxxx
        //
        // 0xe0 = 11100000
        // 0x0f = 00001111
        //
        result += static_cast<char>(0xe0 | ((codepoint >> 12) & 0x0f));
        result += static_cast<char>(0x80 | ((codepoint >> 6) & 0x1f));
        result += static_cast<char>(0x80 | (codepoint & 0x3f));
    }
    else
    {
        // 4-byte codepoints: 000uuuuu zzzzyyyy yyxxxxxx
        // repr: 11110uuu 10uuzzzz 10yyyyyy 10xxxxxx
        //
        // 0xf0 = 11110000
        // 0x07 = 00000111
        //
        result += static_cast<char>(0xf0 | ((codepoint >> 18) & 0x07));
        result += static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f));
        result += static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f));
        result += static_cast<char>(0x80 | (codepoint & 0x3f));
    }
    return result;
}

uint32_t parser::parse_hex(std::string::iterator& it,
                           const std::string::iterator& end, uint32_t place)
{
    uint32_t value = 0;
    while (place > 0)
    {
        if (it == end)
            throw_parse_exception("Unexpected end of unicode sequence");

        if (!is_hex(*it))
            throw_parse_exception("Invalid unicode escape sequence");

        value += place * hex_to_digit(*it++);
        place /= 16;
    }
    return value;
}

std::shared_ptr<base> parser::parse_number(std::string::iterator& it,
                                           const std::string::iterator& end)
{
    auto check_it = it;
    auto check_end = find_end_of_number(it, end);

    auto eat_sign = [&]() {
        if (check_it != end && (*check_it == '-' || *check_it == '+'))
            ++check_it;
    };

    auto check_no_leading_zero = [&]() {
        if (check_it != end && *check_it == '0' && check_it + 1 != check_end
            && check_it[1] != '.')
        {
            throw_parse_exception("Numbers may not have leading zeros");
        }
    };

    auto eat_digits = [&](bool (*check_char)(char)) {
        auto beg = check_it;
        while (check_it != end && check_char(*check_it))
        {
            ++check_it;
            if (check_it != end && *check_it == '_')
            {
                ++check_it;
                if (check_it == end || !check_char(*check_it))
                    throw_parse_exception("Malformed number");
            }
        }

        if (check_it == beg)
            throw_parse_exception("Malformed number");
    };

    auto eat_hex = [&]() { eat_digits(&is_hex); };

    auto eat_numbers = [&]() { eat_digits(&is_number); };

    if (check_it != end && *check_it == '0' && check_it + 1 != check_end
        && (check_it[1] == 'x' || check_it[1] == 'o' || check_it[1] == 'b'))
    {
        ++check_it;
        char base = *check_it;
        ++check_it;
        if (base == 'x')
        {
            eat_hex();
            return parse_int(it, check_it, 16);
        }
        else if (base == 'o')
        {
            auto start = check_it;
            eat_numbers();
            auto val = parse_int(start, check_it, 8, "0");
            it = start;
            return val;
        }
        else // if (base == 'b')
        {
            auto start = check_it;
            eat_numbers();
            auto val = parse_int(start, check_it, 2);
            it = start;
            return val;
        }
    }

    eat_sign();
    check_no_leading_zero();

    if (check_it != end && check_it + 1 != end && check_it + 2 != end)
    {
        if (check_it[0] == 'i' && check_it[1] == 'n' && check_it[2] == 'f')
        {
            auto val = std::numeric_limits<double>::infinity();
            if (*it == '-')
                val = -val;
            it = check_it + 3;
            return make_value(val);
        }
        else if (check_it[0] == 'n' && check_it[1] == 'a'
                 && check_it[2] == 'n')
        {
            auto val = std::numeric_limits<double>::quiet_NaN();
            if (*it == '-')
                val = -val;
            it = check_it + 3;
            return make_value(val);
        }
    }

    eat_numbers();

    if (check_it != end
        && (*check_it == '.' || *check_it == 'e' || *check_it == 'E'))
    {
        bool is_exp = *check_it == 'e' || *check_it == 'E';

        ++check_it;
        if (check_it == end)
            throw_parse_exception("Floats must have trailing digits");

        auto eat_exp = [&]() {
            eat_sign();
            check_no_leading_zero();
            eat_numbers();
        };

        if (is_exp)
            eat_exp();
        else
            eat_numbers();

        if (!is_exp && check_it != end
            && (*check_it == 'e' || *check_it == 'E'))
        {
            ++check_it;
            eat_exp();
        }

        return parse_float(it, check_it);
    }
    else
    {
        return parse_int(it, check_it);
    }
}

std::shared_ptr<value<int64_t>> parser::parse_int(std::string::iterator& it,
                                                  const std::string::iterator& end,
                                                  int base,
                                                  const char* prefix)
{
    std::string v{it, end};
    v = prefix + v;
    v.erase(std::remove(v.begin(), v.end(), '_'), v.end());
    it = end;
    try
    {
        return make_value<int64_t>(std::stoll(v, nullptr, base));
    }
    catch (const std::invalid_argument& ex)
    {
        throw_parse_exception("Malformed number (invalid argument: "
                              + std::string{ex.what()} + ")");
    }
    catch (const std::out_of_range& ex)
    {
        throw_parse_exception("Malformed number (out of range: "
                              + std::string{ex.what()} + ")");
    }
}

std::shared_ptr<value<double>> parser::parse_float(std::string::iterator& it,
                                                   const std::string::iterator& end)
{
    std::string v{it, end};
    v.erase(std::remove(v.begin(), v.end(), '_'), v.end());
    it = end;
    char decimal_point = std::localeconv()->decimal_point[0];
    std::replace(v.begin(), v.end(), '.', decimal_point);
    try
    {
        return make_value<double>(std::stod(v));
    }
    catch (const std::invalid_argument& ex)
    {
        throw_parse_exception("Malformed number (invalid argument: "
                              + std::string{ex.what()} + ")");
    }
    catch (const std::out_of_range& ex)
    {
        throw_parse_exception("Malformed number (out of range: "
                              + std::string{ex.what()} + ")");
    }
}

std::shared_ptr<value<bool>> parser::parse_bool(std::string::iterator& it,
                                                const std::string::iterator& end)
{
    auto eat = make_consumer(it, end, [this]() {
        throw_parse_exception("Attempted to parse invalid boolean value");
    });

    if (*it == 't')
    {
        eat("true");
        return make_value<bool>(true);
    }
    else if (*it == 'f')
    {
        eat("false");
        return make_value<bool>(false);
    }

    eat.error();
    return nullptr;
}

std::string::iterator parser::find_end_of_number(std::string::iterator it,
                                                 std::string::iterator end)
{
    auto ret = std::find_if(it, end, [](char c) {
        return !is_number(c) && c != '_' && c != '.' && c != 'e' && c != 'E'
               && c != '-' && c != '+' && c != 'x' && c != 'o' && c != 'b';
    });
    if (ret != end && ret + 1 != end && ret + 2 != end)
    {
        if ((ret[0] == 'i' && ret[1] == 'n' && ret[2] == 'f')
            || (ret[0] == 'n' && ret[1] == 'a' && ret[2] == 'n'))
        {
            ret = ret + 3;
        }
    }
    return ret;
}

std::string::iterator parser::find_end_of_date(std::string::iterator it,
                                               std::string::iterator end)
{
    auto end_of_date = std::find_if(it, end, [](char c) {
        return !is_number(c) && c != '-';
    });
    if (end_of_date != end && *end_of_date == ' ' && end_of_date + 1 != end
        && is_number(end_of_date[1]))
        end_of_date++;
    return std::find_if(end_of_date, end, [](char c) {
        return !is_number(c) && c != 'T' && c != 'Z' && c != ':'
               && c != '-' && c != '+' && c != '.';
    });
}

local_time parser::read_time(std::string::iterator& it,
                             const std::string::iterator& end)
{
    auto time_end = find_end_of_time(it, end);

    auto eat = make_consumer(
        it, time_end, [&]() { throw_parse_exception("Malformed time"); });

    local_time ltime;

    ltime.hour = eat.eat_digits(2);
    eat(':');
    ltime.minute = eat.eat_digits(2);
    eat(':');
    ltime.second = eat.eat_digits(2);

    int power = 100000;
    if (it != time_end && *it == '.')
    {
        ++it;
        while (it != time_end && is_number(*it))
        {
            ltime.microsecond += power * (*it++ - '0');
            power /= 10;
        }
    }

    if (it != time_end)
        throw_parse_exception("Malformed time");

    return ltime;
}

std::shared_ptr<base> parser::parse_date(std::string::iterator& it,
                                         const std::string::iterator& end)
{
    auto date_end = find_end_of_date(it, end);

    auto eat = make_consumer(
        it, date_end, [&]() { throw_parse_exception("Malformed date"); });

    local_date ldate;
    ldate.year = eat.eat_digits(4);
    eat('-');
    ldate.month = eat.eat_digits(2);
    eat('-');
    ldate.day = eat.eat_digits(2);

    if (it == date_end)
        return make_value(ldate);

    eat.eat_or('T', ' ');

    local_datetime ldt;
    static_cast<local_date&>(ldt) = ldate;
    static_cast<local_time&>(ldt) = read_time(it, date_end);

    if (it == date_end)
        return make_value(ldt);

    offset_datetime dt;
    static_cast<local_datetime&>(dt) = ldt;

    int hoff = 0;
    int moff = 0;
    if (*it == '+' || *it == '-')
    {
        auto plus = *it == '+';
        ++it;

        hoff = eat.eat_digits(2);
        dt.hour_offset = (plus) ? hoff : -hoff;
        eat(':');
        moff = eat.eat_digits(2);
        dt.minute_offset = (plus) ? moff : -moff;
    }
    else if (*it == 'Z')
    {
        ++it;
    }

    if (it != date_end)
        throw_parse_exception("Malformed date");

    return make_value(dt);
}

std::shared_ptr<base> parser::parse_array(std::string::iterator& it,
                                          std::string::iterator& end)
{
    // this gets ugly because of the "homogeneity" restriction:
    // arrays can either be of only one type, or contain arrays
    // (each of those arrays could be of different types, though)
    //
    // because of the latter portion, we don't really have a choice
    // but to represent them as arrays of base values...
    ++it;

    // ugh---have to read the first value to determine array type...
    skip_whitespace_and_comments(it, end);

    // edge case---empty array
    if (*it == ']')
    {
        ++it;
        return make_array();
    }

    auto val_end = std::find_if(
        it, end, [](char c) { return c == ',' || c == ']' || c == '#'; });
    parse_type type = determine_value_type(it, val_end);
    switch (type)
    {
        case parse_type::STRING:
            return parse_value_array<std::string>(it, end);
        case parse_type::LOCAL_TIME:
            return parse_value_array<local_time>(it, end);
        case parse_type::LOCAL_DATE:
            return parse_value_array<local_date>(it, end);
        case parse_type::LOCAL_DATETIME:
            return parse_value_array<local_datetime>(it, end);
        case parse_type::OFFSET_DATETIME:
            return parse_value_array<offset_datetime>(it, end);
        case parse_type::INT:
            return parse_value_array<int64_t>(it, end);
        case parse_type::FLOAT:
            return parse_value_array<double>(it, end);
        case parse_type::BOOL:
            return parse_value_array<bool>(it, end);
        case parse_type::ARRAY:
            return parse_object_array<array>(&parser::parse_array, '[', it,
                                             end);
        case parse_type::INLINE_TABLE:
            return parse_object_array<table_array>(
                &parser::parse_inline_table, '{', it, end);
        default:
            throw_parse_exception("Unable to parse array");
    }
}

std::shared_ptr<table> parser::parse_inline_table(std::string::iterator& it,
                                                  std::string::iterator& end)
{
    auto tbl = make_table();
    do
    {
        ++it;
        if (it == end)
            throw_parse_exception("Unterminated inline table");

        consume_whitespace(it, end);
        if (it != end && *it != '}')
        {
            parse_key_value(it, end, tbl.get());
            consume_whitespace(it, end);
        }
    } while (*it == ',');

    if (it == end || *it != '}')
        throw_parse_exception("Unterminated inline table");

    ++it;
    consume_whitespace(it, end);

    return tbl;
}

void parser::skip_whitespace_and_comments(std::string::iterator& start,
                                          std::string::iterator& end)
{
    consume_whitespace(start, end);
    while (start == end || *start == '#')
    {
        if (!detail::getline(input_, line_))
            throw_parse_exception("Unclosed array");
        line_number_++;
        start = line_.begin();
        end = line_.end();
        consume_whitespace(start, end);
    }
}

bool parser::is_time(const std::string::iterator& it,
                     const std::string::iterator& end)
{
    auto time_end = find_end_of_time(it, end);
    auto len = std::distance(it, time_end);

    if (len < 8)
        return false;

    if (it[2] != ':' || it[5] != ':')
        return false;

    if (len > 8)
        return it[8] == '.' && len > 9;

    return true;
}

option<parser::parse_type> parser::date_type(const std::string::iterator& it,
                                             const std::string::iterator& end)
{
    auto date_end = find_end_of_date(it, end);
    auto len = std::distance(it, date_end);

    if (len < 10)
        return {};

    if (it[4] != '-' || it[7] != '-')
        return {};

    if (len >= 19 && (it[10] == 'T' || it[10] == ' ')
        && is_time(it + 11, date_end))
    {
        // datetime type
        auto time_end = find_end_of_time(it + 11, date_end);
        if (time_end == date_end)
            return {parse_type::LOCAL_DATETIME};
        else
            return {parse_type::OFFSET_DATETIME};
    }
    else if (len == 10)
    {
        // just a regular date
        return {parse_type::LOCAL_DATE};
    }

    return {};
}

std::shared_ptr<table> parse_file(const std::string& filename)
{
#if defined(BOOST_NOWIDE_FSTREAM_INCLUDED_HPP)
    boost::nowide::ifstream file{filename.c_str()};
#elif defined(NOWIDE_FSTREAM_INCLUDED_HPP)
    nowide::ifstream file{filename.c_str()};
#else
    std::ifstream file{filename};
#endif
    if (!file.is_open())
        throw parse_exception{filename + " could not be opened for parsing"};
    parser p{file};
    return p.parse();
}

void toml_writer::visit(const table& t, bool in_array)
{
    write_table_header(in_array);
    std::vector<std::string> values;
    std::vector<std::string> tables;

    for (const auto& i : t)
    {
        if (i.second->is_table() || i.second->is_table_array())
        {
            tables.push_back(i.first);
        }
        else
        {
            values.push_back(i.first);
        }
    }

    for (unsigned int i = 0; i < values.size(); ++i)
    {
        path_.push_back(values[i]);

        if (i > 0)
            endline();

        write_table_item_header(*t.get(values[i]));
        t.get(values[i])->accept(*this, false);
        path_.pop_back();
    }

    for (unsigned int i = 0; i < tables.size(); ++i)
    {
        path_.push_back(tables[i]);

        if (values.size() > 0 || i > 0)
            endline();

        write_table_item_header(*t.get(tables[i]));
        t.get(tables[i])->accept(*this, false);
        path_.pop_back();
    }

    endline();
}

void toml_writer::visit(const array& a, bool)
{
    write("[");

    for (unsigned int i = 0; i < a.get().size(); ++i)
    {
        if (i > 0)
            write(", ");

        if (a.get()[i]->is_array())
        {
            a.get()[i]->as_array()->accept(*this, true);
        }
        else
        {
            a.get()[i]->accept(*this, true);
        }
    }

    write("]");
}

void toml_writer::visit(const table_array& t, bool)
{
    for (unsigned int j = 0; j < t.get().size(); ++j)
    {
        if (j > 0)
            endline();

        t.get()[j]->accept(*this, true);
    }

    endline();
}

std::string toml_writer::escape_string(const std::string& str)
{
    std::string res;
    for (auto it = str.begin(); it != str.end(); ++it)
    {
        if (*it == '\b')
        {
            res += "\\b";
        }
        else if (*it == '\t')
        {
            res += "\\t";
        }
        else if (*it == '\n')
        {
            res += "\\n";
        }
        else if (*it == '\f')
        {
            res += "\\f";
        }
        else if (*it == '\r')
        {
            res += "\\r";
        }
        else if (*it == '"')
        {
            res += "\\\"";
        }
        else if (*it == '\\')
        {
            res += "\\\\";
        }
        else if (static_cast<uint32_t>(*it) <= UINT32_C(0x001f))
        {
            res += "\\u";
            std::stringstream ss;
            ss << std::hex << static_cast<uint32_t>(*it);
            res += ss.str();
        }
        else
        {
            res += *it;
        }
    }
    return res;
}

void toml_writer::write(const value<double>& v)
{
    std::stringstream ss;
    ss << std::showpoint
       << std::setprecision(std::numeric_limits<double>::max_digits10)
       << v.get();

    auto double_str = ss.str();
    auto pos = double_str.find("e0");
    if (pos != std::string::npos)
        double_str.replace(pos, 2, "e");
    pos = double_str.find("e-0");
    if (pos != std::string::npos)
        double_str.replace(pos, 3, "e-");

    stream_ << double_str;
    has_naked_endline_ = false;
}

void toml_writer::write_table_header(bool in_array)
{
    if (!path_.empty())
    {
        indent();

        write("[");

        if (in_array)
        {
            write("[");
        }

        for (unsigned int i = 0; i < path_.size(); ++i)
        {
            if (i > 0)
            {
                write(".");
            }

            if (path_[i].find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcde"
                                           "fghijklmnopqrstuvwxyz0123456789"
                                           "_-")
                == std::string::npos)
            {
                write(path_[i]);
            }
            else
            {
                write("\"");
                write(escape_string(path_[i]));
                write("\"");
            }
        }

        if (in_array)
        {
            write("]");
        }

        write("]");
        endline();
    }
}

void toml_writer::write_table_item_header(const base& b)
{
    if (!b.is_table() && !b.is_table_array())
    {
        indent();

        if (path_.back().find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcde"
                                           "fghijklmnopqrstuvwxyz0123456789"
                                           "_-")
            == std::string::npos)
        {
            write(path_.back());
        }
        else
        {
            write("\"");
            write(escape_string(path_.back()));
            write("\"");
        }

        write(" = ");
    }
}

} // namespace cpptoml
