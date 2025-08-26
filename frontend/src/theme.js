import { extendTheme } from "@chakra-ui/react";

const theme = extendTheme({
  config: {
    initialColorMode: "dark",
    useSystemColorMode: false,
  },
  colors: {
    brand: {
      50: "#f0f9ff",
      100: "#e0f2fe",
      200: "#bae6fd",
      300: "#7dd3fc",
      400: "#38bdf8",
      500: "#0ea5e9",
      600: "#0284c7",
      700: "#0369a1",
      800: "#075985",
      900: "#0c4a6e",
    },
    accent: {
      50: "#fdf4ff",
      100: "#fae8ff",
      200: "#f5d0fe",
      300: "#f0abfc",
      400: "#e879f9",
      500: "#d946ef",
      600: "#c026d3",
      700: "#a21caf",
      800: "#86198f",
      900: "#701a75",
    },
    success: {
      50: "#f0fdf4",
      100: "#dcfce7",
      200: "#bbf7d0",
      300: "#86efac",
      400: "#4ade80",
      500: "#22c55e",
      600: "#16a34a",
      700: "#15803d",
      800: "#166534",
      900: "#14532d",
    },
    dark: {
      50: "#f8fafc",
      100: "#f1f5f9",
      200: "#e2e8f0",
      300: "#cbd5e1",
      400: "#94a3b8",
      500: "#64748b",
      600: "#475569",
      700: "#334155",
      800: "#1e293b",
      900: "#0f172a",
    }
  },
  fonts: {
    heading: "'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif",
    body: "'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif",
  },
  fontSizes: {
    xs: "0.75rem",
    sm: "0.875rem",
    md: "1rem",
    lg: "1.125rem",
    xl: "1.25rem",
    "2xl": "1.5rem",
    "3xl": "1.875rem",
    "4xl": "2.25rem",
    "5xl": "3rem",
    "6xl": "3.75rem",
    "7xl": "4.5rem",
    "8xl": "6rem",
    "9xl": "8rem",
  },
  shadows: {
    glass: "0 8px 32px 0 rgba(31, 38, 135, 0.37)",
    glow: "0 0 20px rgba(56, 189, 248, 0.3)",
    glowHover: "0 0 30px rgba(56, 189, 248, 0.5)",
    card: "0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)",
    cardHover: "0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04)",
  },
  styles: {
    global: {
      body: {
        bg: "linear-gradient(135deg, #0f172a 0%, #1e293b 50%, #334155 100%)",
        minHeight: "100vh",
      },
    },
  },
  components: {
    Button: {
      baseStyle: {
        fontWeight: "600",
        borderRadius: "xl",
        transition: "all 0.3s ease",
        _focus: {
          boxShadow: "none",
        },
      },
      variants: {
        solid: {
          bg: "linear-gradient(135deg, brand.500 0%, brand.600 100%)",
          color: "white",
          border: "1px solid transparent",
          _hover: {
            bg: "linear-gradient(135deg, brand.600 0%, brand.700 100%)",
            transform: "translateY(-2px)",
            boxShadow: "glowHover",
            _disabled: {
              bg: "linear-gradient(135deg, brand.500 0%, brand.600 100%)",
              transform: "none",
              boxShadow: "none",
            },
          },
          _active: {
            transform: "translateY(0)",
          },
        },
        gradient: {
          bg: "linear-gradient(135deg, accent.500 0%, brand.500 100%)",
          color: "white",
          border: "1px solid transparent",
          _hover: {
            bg: "linear-gradient(135deg, accent.600 0%, brand.600 100%)",
            transform: "translateY(-2px)",
            boxShadow: "glowHover",
          },
          _active: {
            transform: "translateY(0)",
          },
        },
        glass: {
          bg: "rgba(255, 255, 255, 0.1)",
          backdropFilter: "blur(10px)",
          border: "1px solid rgba(255, 255, 255, 0.2)",
          color: "white",
          _hover: {
            bg: "rgba(255, 255, 255, 0.15)",
            transform: "translateY(-2px)",
            boxShadow: "glass",
          },
          _active: {
            transform: "translateY(0)",
          },
        },
      },
    },
    Card: {
      baseStyle: {
        container: {
          borderRadius: "2xl",
          transition: "all 0.3s ease",
          border: "1px solid rgba(255, 255, 255, 0.1)",
          backdropFilter: "blur(10px)",
          bg: "rgba(15, 23, 42, 0.8)",
        },
      },
      variants: {
        glass: {
          container: {
            bg: "rgba(255, 255, 255, 0.05)",
            backdropFilter: "blur(20px)",
            border: "1px solid rgba(255, 255, 255, 0.1)",
            boxShadow: "glass",
            _hover: {
              bg: "rgba(255, 255, 255, 0.08)",
              transform: "translateY(-4px)",
              boxShadow: "cardHover",
            },
          },
        },
        gradient: {
          container: {
            bg: "linear-gradient(135deg, rgba(15, 23, 42, 0.9) 0%, rgba(30, 41, 59, 0.9) 100%)",
            border: "1px solid rgba(56, 189, 248, 0.2)",
            _hover: {
              border: "1px solid rgba(56, 189, 248, 0.4)",
              transform: "translateY(-4px)",
              boxShadow: "glow",
            },
          },
        },
      },
    },
    Badge: {
      baseStyle: {
        borderRadius: "full",
        fontWeight: "600",
        px: 3,
        py: 1,
      },
      variants: {
        glass: {
          bg: "rgba(255, 255, 255, 0.1)",
          color: "white",
          backdropFilter: "blur(10px)",
          border: "1px solid rgba(255, 255, 255, 0.2)",
        },
        gradient: {
          bg: "linear-gradient(135deg, brand.500 0%, accent.500 100%)",
          color: "white",
        },
      },
    },
    Input: {
      variants: {
        glass: {
          field: {
            bg: "rgba(255, 255, 255, 0.05)",
            backdropFilter: "blur(10px)",
            border: "1px solid rgba(255, 255, 255, 0.1)",
            borderRadius: "xl",
            color: "white",
            _placeholder: {
              color: "gray.400",
            },
            _hover: {
              border: "1px solid rgba(56, 189, 248, 0.3)",
            },
            _focus: {
              border: "1px solid rgba(56, 189, 248, 0.5)",
              boxShadow: "0 0 0 1px rgba(56, 189, 248, 0.2)",
            },
          },
        },
      },
    },
    Select: {
      variants: {
        glass: {
          field: {
            bg: "rgba(255, 255, 255, 0.05)",
            backdropFilter: "blur(10px)",
            border: "1px solid rgba(255, 255, 255, 0.1)",
            borderRadius: "xl",
            color: "white",
            _hover: {
              border: "1px solid rgba(56, 189, 248, 0.3)",
            },
            _focus: {
              border: "1px solid rgba(56, 189, 248, 0.5)",
              boxShadow: "0 0 0 1px rgba(56, 189, 248, 0.2)",
            },
          },
        },
      },
    },
  },
});

export default theme;
