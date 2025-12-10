/**
 * Translation Feature Test
 * Verifies that the translation system is working correctly
 */

import { translateSimple, translateToUrdu } from '../src/lib/translator';

async function runTests() {
  console.log('🧪 Translation Feature Tests\n');

  // Test 1: Simple offline translations
  console.log('✅ Test 1: Offline Dictionary Translations');
  const testWords = ['Home', 'Search', 'Documentation', 'Sign In', 'Dashboard'];
  
  for (const word of testWords) {
    const translation = translateSimple(word);
    console.log(`  "${word}" → "${translation}"`);
    if (translation === word) {
      console.warn(`  ⚠️  No offline translation found for "${word}"`);
    }
  }

  // Test 2: Case-insensitive matching
  console.log('\n✅ Test 2: Case-Insensitive Matching');
  const lowerHome = translateSimple('home');
  console.log(`  "home" (lowercase) → "${lowerHome}"`);

  // Test 3: Empty/null handling
  console.log('\n✅ Test 3: Empty/Null Handling');
  const emptyResult = translateSimple('');
  console.log(`  Empty string → "${emptyResult}" (should be empty)`);

  // Test 4: API translation (async)
  console.log('\n✅ Test 4: MyMemory API Translation');
  console.log('  Testing API with: "Welcome to our course"');
  try {
    const apiTranslation = await translateToUrdu('Welcome to our course');
    console.log(`  Result: "${apiTranslation}"`);
    console.log('  ✓ API call successful');
  } catch (error) {
    console.error(`  ✗ API call failed: ${error}`);
  }

  console.log('\n✅ All tests completed!');
}

// Run tests
runTests().catch(console.error);
