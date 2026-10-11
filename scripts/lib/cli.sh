# Shared operator CLI parser. Bash 4+; no eval and no shell word splitting.
# Kept identical in Primitive and Robot so each repository is self-contained.
cli_init() {
  CLI_USAGE="$1"
  CLI_PASSTHROUGH=0
  CLI_ARGS=()
  CLI_NAMES=()
  CLI_VARS=()
  CLI_KINDS=()
  CLI_DOCS=()
}
cli_option() {
  CLI_NAMES+=("$1"); CLI_VARS+=("$2"); CLI_KINDS+=(value); CLI_DOCS+=("$3")
}
cli_flag() {
  CLI_NAMES+=("$1"); CLI_VARS+=("$2"); CLI_KINDS+=(flag); CLI_DOCS+=("$3")
}
cli_die() { echo "error: $*" >&2; exit 2; }
cli_help() {
  printf 'Usage: %s\n\nOptions:\n' "$CLI_USAGE"
  local i
  for ((i = 0; i < ${#CLI_NAMES[@]}; i++)); do
    printf '  %-32s %s\n' "${CLI_NAMES[$i]}" "${CLI_DOCS[$i]}"
  done
  printf '  %-32s %s\n' '--help, -h' 'Show help without running Docker or contacting hardware'
}
cli_parse() {
  local key value found i
  while (($#)); do
    case "$1" in
      -h|--help) cli_help; exit 0 ;;
      --) shift; CLI_ARGS+=("$@"); break ;;
    esac
    key="${1%%=*}"
    found=0
    for ((i = 0; i < ${#CLI_NAMES[@]}; i++)); do
      [[ "$key" == "${CLI_NAMES[$i]}" ]] || continue
      found=1
      if [[ "${CLI_KINDS[$i]}" == flag ]]; then
        [[ "$1" != *=* ]] || cli_die "$key takes no value"
        printf -v "${CLI_VARS[$i]}" '%s' 1
        shift
      else
        if [[ "$1" == *=* ]]; then
          value="${1#*=}"; shift
        else
          (($# >= 2)) || cli_die "$key requires a value"
          [[ "$2" != --* && "$2" != -h ]] || cli_die "$key requires a value"
          value="$2"; shift 2
        fi
        [[ -n "$value" ]] || cli_die "$key requires a non-empty value"
        printf -v "${CLI_VARS[$i]}" '%s' "$value"
      fi
      break
    done
    if ((found == 0)); then
      if [[ "${CLI_PASSTHROUGH:-0}" != 1 && "$1" == -* ]]; then
        cli_die "unknown argument: $1"
      fi
      CLI_ARGS+=("$1"); shift
    fi
  done
}
cli_no_args() { ((${#CLI_ARGS[@]} == 0)) || cli_die "unexpected arguments: ${CLI_ARGS[*]}"; }
cli_port() {
  [[ "$2" =~ ^[0-9]{4,5}$ ]] && ((10#$2 >= 1024 && 10#$2 <= 65535)) ||
    cli_die "$1 must be in 1024..65535"
}
cli_network() {
  [[ "$1" == host || "$1" == bridge ]] || cli_die "--network-mode must be host or bridge"
}
